#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace PyroWaveLink {
constexpr int minimumKbps = 5000;
constexpr int maximumKbps = 3000000;
constexpr int durationMs = 2000;
constexpr int windowMs = 100;
constexpr int windowCount = durationMs / windowMs;

// PyroWave turns lost detail into blur: the receiver releases frames on time
// without it and the pacer does not buffer for it, so under 2% loss is usable
// (one bad 100 ms window above 5% is bunched, visible loss). Packets the host
// could not send count as lost; below 98% sent, the host itself is overloaded.
// A 0.1% limit failed a whole 1.25 Gbps step for 384 unsent packets and
// collapsed calibration to 480 Mbps.
constexpr double lossLimitPercent = 2.0;
constexpr double windowLossLimitPercent = 5.0;
constexpr double minimumSentShare = 0.98;

struct Result {
    int requestedKbps = 0;
    uint32_t expected = 0;
    uint32_t sent = 0;
    uint32_t received = 0;
    double senderMs = 0;
    double lossPercent = 100;
    double worstWindowLossPercent = 100;
    double delayP99Ms = 0;
    double delayGrowthMs = 0;
    bool kernelArrivalTimestamps = false;
    double receiverReadDelayP99Ms = 0;
    // Host diagnostics (Vibeshine): packets retried after a full send buffer
    // and the last send error. Older hosts report neither.
    uint32_t hostSendRetries = 0;
    int hostLastSendError = 0;

    bool hostSentEnough() const {
        return expected > 0 && sent <= expected && sent >= expected * minimumSentShare;
    }

    const char* failureReason() const {
        if (!expected) return "no probe completed";
        if (!hostSentEnough()) return "host did not send enough probe packets";
        if (!received) return "no UDP packets received";
        if (received > sent) return "invalid received packet count";
        if (!std::isfinite(senderMs) || !std::isfinite(lossPercent) ||
            !std::isfinite(worstWindowLossPercent) || !std::isfinite(delayP99Ms) ||
            !std::isfinite(delayGrowthMs)) return "invalid timing measurement";
        if (senderMs < durationMs * 0.98 || senderMs > durationMs * 1.02) return "host probe missed its sending duration";
        if (lossPercent >= lossLimitPercent) return "packet loss exceeds the limit";
        if (worstWindowLossPercent >= windowLossLimitPercent) return "bursty packet loss exceeds the limit";
        if (delayP99Ms > 4.0) return "packet delivery variation exceeds 4 ms";
        if (delayGrowthMs > 2.0) return "packet delivery delay grows by more than 2 ms";
        return "stable";
    }

    // Capacity qualification retains loss, sender pacing and queue-growth
    // limits. Transit jitter is a separate smoothness warning, not proof that
    // the codec/device is unsupported or that lowering bitrate will help.
    bool capacityQualified() const {
        return hostSentEnough() && received > 0 && received <= sent &&
               std::isfinite(delayP99Ms) && std::isfinite(delayGrowthMs) &&
               senderMs >= durationMs * 0.98 && senderMs <= durationMs * 1.02 &&
               lossPercent < lossLimitPercent && worstWindowLossPercent < windowLossLimitPercent &&
               delayGrowthMs <= 2.0;
    }

    bool stable() const {
        return capacityQualified() && delayP99Ms <= 4.0;
    }
};

// Unique sequence IDs make reordering harmless and prevent duplicate packets
// from hiding loss. Delay is relative to the minimum transit in this run, so
// host/client clock offsets never enter the grade.
inline void summarize(Result& result, const std::vector<int64_t>& arrivalsUs)
{
    std::vector<double> transit;
    double first = 0, last = 0;
    int firstCount = 0, lastCount = 0;
    result.received = 0;
    result.worstWindowLossPercent = 0;
    for (int window = 0; window < windowCount; ++window) {
        const uint32_t begin = uint64_t(result.expected) * window / windowCount;
        const uint32_t end = uint64_t(result.expected) * (window + 1) / windowCount;
        uint32_t count = 0;
        for (uint32_t seq = begin; seq < end && seq < arrivalsUs.size(); ++seq) {
            if (arrivalsUs[seq] < 0) continue;
            ++count;
            // Sender uses this same 1 ms packet-group schedule.
            const auto tick = (uint64_t(seq + 1) * durationMs + result.expected - 1) / result.expected - 1;
            const double delta = arrivalsUs[seq] / 1000.0 - tick;
            transit.push_back(delta);
            if (window == 0) { first += delta; ++firstCount; }
            if (window == windowCount - 1) { last += delta; ++lastCount; }
        }
        result.received += count;
        if (end > begin) result.worstWindowLossPercent = (std::max)(result.worstWindowLossPercent,
            100.0 * (end - begin - count) / (end - begin));
    }
    result.lossPercent = result.expected ? 100.0 * (result.expected - result.received) / result.expected : 100;
    if (transit.empty()) return;
    std::sort(transit.begin(), transit.end());
    result.delayP99Ms = transit[(transit.size() - 1) * 99 / 100] - transit.front();
    result.delayGrowthMs = firstCount && lastCount ? last / lastCount - first / firstCount : INFINITY;
}

inline int rounded(double kbps) { return int(kbps / minimumKbps) * minimumKbps; }

// First test the requested rate; grow until a failure, then refine that bracket.
// A final 5% margin is measured twice afresh. Failed confirmation lowers the
// bracket, so a noisy/high-loss path can never inherit an earlier passing rate.
template<class Probe, class Cancelled>
Result search(int targetKbps, int capKbps, Probe probe, Cancelled cancelled,
              bool requireLowJitter = true)
{
    const auto qualified = [requireLowJitter](const Result& result) {
        return requireLowJitter ? result.stable() : result.capacityQualified();
    };
    const int cap = (std::min)(maximumKbps, rounded(capKbps));
    if (cap < minimumKbps) return {};
    int low = 0, high = cap + minimumKbps;
    int next = std::clamp(rounded(targetKbps), minimumKbps, cap);
    Result best;
    for (int attempt = 0; attempt < 40 && !cancelled(); ++attempt) {
        const auto result = probe(next);
        best = result;
        if (cancelled()) return {};
        if (qualified(result)) {
            best = result;
            low = next;
            if (low == cap || high - low <= minimumKbps) break;
            next = high <= cap ? rounded((low + high) / 2.0) :
                (std::min)(cap, (std::max)(low + minimumKbps, rounded(low * 1.25)));
        }
        else {
            high = next;
            if (high - low <= minimumKbps) break;
            next = (std::max)(minimumKbps, rounded((low + high) / 2.0));
        }
    }
    if (!low) return best;
    next = (std::max)(minimumKbps, rounded(low * 0.95));
    for (int attempt = 0; attempt < 32 && !cancelled(); ++attempt) {
        best = probe(next);
        if (cancelled()) return {};
        if (qualified(best)) {
            best = probe(next);
            if (!cancelled() && qualified(best)) return best;
        }
        if (next == minimumKbps) break;
        next = (std::max)(minimumKbps, rounded(next * 0.8));
    }
    return cancelled() ? Result{} : best;
}

// Frame pacing. The capacity probe spreads each millisecond's share evenly, so
// it never shows whether the receive path can absorb a whole frame arriving
// back-to-back. Frame-shaped probes (NvHTTP::probePyroWaveUdp burstFps) send
// the same packets as video frames on the stream pacer's 1 ms groups. A USB
// 2.5GbE dock lost whole 12-16 packet USB transfers above ~2.2 Gbps although
// its link reported 2.5 Gbps; the host then paces at the pace found here.
constexpr int paceStepKbps = 50000;
// The same loss limits as capacity: below 2% a faster pace (sooner whole
// frames) is worth more than the missing detail.
constexpr double paceLossPercent = lossLimitPercent;
constexpr double paceWindowLossPercent = windowLossLimitPercent;

// Only loss and the host's sending duration grade a frame-shaped probe: its
// delivery timing is intentionally bursty, not the capacity probe's schedule.
inline bool paceQualified(const Result& result)
{
    return result.hostSentEnough() && result.received > 0 && result.received <= result.sent &&
           std::isfinite(result.senderMs) && std::isfinite(result.lossPercent) &&
           result.senderMs >= durationMs * 0.98 && result.senderMs <= durationMs * 1.02 &&
           result.lossPercent < paceLossPercent && result.worstWindowLossPercent < paceWindowLossPercent;
}

struct PaceResult {
    int paceKbps = 0;      // Zero: not determined; the host keeps its default.
    bool lossless = false; // True: within the loss limit. False: none was; paceKbps lost least.
    Result probe;          // Measurement behind paceKbps.
};

// Highest pace, between floorKbps (a frame must still fit the frame interval)
// and the link, within the loss limit; if none is, the pace that lost least
// (the faster on ties). The link is tried first. Below it, a bisected pass
// leaves a 5% margin that must pass twice afresh, stepping down on failure.
template<class Probe, class Cancelled>
PaceResult searchPace(int linkKbps, int floorKbps, Probe probe, Cancelled cancelled)
{
    const auto stepDown = [](double kbps) { return int(kbps / paceStepKbps) * paceStepKbps; };
    const int cap = stepDown(linkKbps);
    if (cap < paceStepKbps) return {};
    const int floor = std::clamp(int(std::ceil(double(floorKbps) / paceStepKbps)) * paceStepKbps,
                                 paceStepKbps, cap);
    PaceResult fewest;
    const auto passes = [&](int pace) {
        const Result result = probe(pace);
        if (!fewest.paceKbps || result.lossPercent < fewest.probe.lossPercent ||
            (result.lossPercent == fewest.probe.lossPercent && pace > fewest.paceKbps)) {
            fewest = {pace, false, result};
        }
        return paceQualified(result);
    };
    const auto confirmed = [&](int pace) -> PaceResult {
        Result last;
        for (int run = 0; run < 2; ++run) {
            if (cancelled()) return {};
            last = probe(pace);
            if (!paceQualified(last)) return {};
        }
        return {pace, true, last};
    };
    if (passes(cap)) {
        if (cancelled()) return {};
        const auto atLink = confirmed(cap);
        if (atLink.lossless || cancelled()) return cancelled() ? PaceResult{} : atLink;
    }
    if (cancelled()) return {};
    int low = 0, high = cap;
    if (floor < cap) {
        if (!passes(floor)) return cancelled() ? PaceResult{} : fewest;
        low = floor;
        for (int attempt = 0; attempt < 16 && high - low > paceStepKbps && !cancelled(); ++attempt) {
            const int next = (std::max)(low + paceStepKbps, stepDown((low + high) / 2.0));
            if (next >= high) break;
            if (passes(next)) low = next;
            else high = next;
        }
    }
    if (cancelled() || !low) return cancelled() ? PaceResult{} : fewest;
    for (int pace = (std::max)(floor, stepDown(low * 0.95)); !cancelled();
         pace = (std::max)(floor, stepDown(pace * 0.9))) {
        const auto result = confirmed(pace);
        if (result.lossless) return result;
        if (pace == floor) break;
    }
    return cancelled() ? PaceResult{} : fewest;
}
} // namespace PyroWaveLink
