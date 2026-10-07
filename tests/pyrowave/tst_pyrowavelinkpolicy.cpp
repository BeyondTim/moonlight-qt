#include "../../app/streaming/video/pyrowave/pyrowavelinkpolicy.h"
#include "../../app/streaming/video/pyrowave/pyrowavebandwidth.h"
#include "../../app/streaming/video/pyrowave/pyrowavecalibrationpolicy.h"
#include <cstdio>
#include <cstdlib>
#include <string>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "Failed line %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)

using namespace PyroWaveLink;
Result sample(int kbps, bool passing) {
    Result r;
    r.requestedKbps = kbps;
    r.expected = r.sent = r.received = 10000;
    r.senderMs = durationMs;
    r.lossPercent = passing ? 0 : 2;
    r.worstWindowLossPercent = passing ? 0 : 2;
    return r;
}
int main() {
    std::vector<int> tried;
    auto run = [&](int start, int limit) {
        tried.clear();
        return search(start, maximumKbps, [&](int rate) {
            tried.push_back(rate);
            return sample(rate, rate <= limit);
        }, [] { return false; });
    };
    auto r = run(500000, 1000000);
    CHECK(tried.front() == 500000);
    CHECK(*std::max_element(tried.begin(), tried.end()) > 1000000);
    CHECK(r.requestedKbps >= 940000 && r.requestedKbps <= 950000);
    CHECK(tried[tried.size()-1] == r.requestedKbps && tried[tried.size()-2] == r.requestedKbps);
    r = run(2000000, 800000);
    CHECK(tried.front() == 2000000 && r.requestedKbps <= 760000 && r.requestedKbps >= 750000);
    CHECK(!run(500000, 0).stable());
    // A failed search retains the last floor probe so the UI can name its
    // actual failure instead of claiming that UDP may be blocked.
    auto failed = run(500000, 0);
    CHECK(failed.requestedKbps == minimumKbps && failed.expected == 10000);
    CHECK(std::string(failed.failureReason()) == "packet loss exceeds the limit");
    failed = search(500000, maximumKbps, [](int rate) {
        auto measured = sample(rate, true);
        measured.delayP99Ms = 20;
        return measured;
    }, [] { return false; });
    CHECK(!failed.stable() && failed.received == failed.expected);
    CHECK(std::string(failed.failureReason()) == "packet delivery variation exceeds 4 ms");
    // Jitter without congestion must not hide local format support. Capacity
    // search still finds a loss boundary, leaves margin and confirms twice.
    tried.clear();
    auto capacity = search(500000, maximumKbps, [&](int rate) {
        tried.push_back(rate);
        auto measured = sample(rate, rate <= 800000);
        measured.delayP99Ms = 16;
        return measured;
    }, [] { return false; }, false);
    CHECK(capacity.capacityQualified() && !capacity.stable());
    CHECK(capacity.requestedKbps >= 750000 && capacity.requestedKbps <= 760000);
    CHECK(tried.back() == capacity.requestedKbps && tried[tried.size()-2] == capacity.requestedKbps);
    CHECK(!search(500000, maximumKbps, [](int rate) { return sample(rate, false); },
                  [] { return false; }, false).capacityQualified());
    CHECK(!search(500000, maximumKbps, [](int rate) {
        auto measured = sample(rate, true);
        measured.delayGrowthMs = 3;
        return measured;
    }, [] { return false; }, false).capacityQualified());
    CHECK(run(500000, maximumKbps).requestedKbps == 2850000);
    CHECK(run(minimumKbps, maximumKbps).requestedKbps == 2850000);
    // Ceiling-first search keeps the same 5 Mbps boundary and confirmations,
    // but needs only three probes when the known ceiling is sustainable.
    CHECK(run(500000, maximumKbps).requestedKbps == 2850000);
    int oldAttempts = tried.size();
    CHECK(run(maximumKbps, maximumKbps).requestedKbps == 2850000);
    CHECK(tried.size() == 3 && oldAttempts > int(tried.size()));
    std::printf("UDP at a sustainable 3 Gbps ceiling: %d -> %zu probes (%.1f -> %.1f seconds of sending)\n",
                oldAttempts, tried.size(), oldAttempts * durationMs / 1000.0,
                tried.size() * durationMs / 1000.0);
    // Unknown bottlenecks still get a measured boundary, never an interface-speed guess.
    CHECK(run(maximumKbps, 800000).requestedKbps == 760000);
    CHECK(tried.size() <= 13);
    // Severe random loss at every rate must never produce a suggestion.
    CHECK(!search(500000, maximumKbps, [](int rate) { return sample(rate, false); }, [] { return false; }).stable());
    // A route that deteriorates during final confirmation must back off again.
    int count = 0;
    r = search(500000, 500000, [&](int rate) {
        ++count;
        return sample(rate, count == 1 || rate <= 350000);
    }, [] { return false; });
    CHECK(r.stable() && r.requestedKbps <= 350000);
    bool stop = false;
    CHECK(!search(500000, maximumKbps, [&](int rate) { stop = true; return sample(rate, true); }, [&] { return stop; }).stable());

    Result clean = sample(100000, true);
    std::vector<int64_t> times(clean.expected);
    for (uint32_t seq = 0; seq < clean.expected; ++seq) {
        const auto tick = (uint64_t(seq+1) * durationMs + clean.expected - 1) / clean.expected - 1;
        times[seq] = 123456789 + tick * 1000;
    }
    summarize(clean, times);
    CHECK(clean.stable() && clean.delayP99Ms < 0.001);
    // PyroWave tolerates a short burst (8 packets, 1.6% of one window)...
    for (int i = 0; i < 8; ++i) times[i] = -1;
    summarize(clean, times);
    CHECK(clean.lossPercent < 0.1 && clean.worstWindowLossPercent > 1 && clean.stable());
    // ...but not loss bunched into one window above 5%, even under 2% overall.
    for (int i = 0; i < 30; ++i) times[i] = -1;
    summarize(clean, times);
    CHECK(clean.lossPercent < 2 && clean.worstWindowLossPercent > 5 && !clean.stable());
    CHECK(std::string(clean.failureReason()) == "bursty packet loss exceeds the limit");
    for (int i = 0; i < 30; ++i) {
        const auto tick = (uint64_t(i+1) * durationMs + clean.expected - 1) / clean.expected - 1;
        times[i] = 123456789 + tick * 1000;
    }
    // Tail loss is counted against the sender's expected count.
    times.assign(clean.expected, -1);
    summarize(clean, times);
    CHECK(clean.lossPercent == 100 && !clean.stable());
    // Lossless queue growth must fail too; a constant clock offset is harmless.
    for (uint32_t seq = 0; seq < clean.expected; ++seq) times[seq] = 100000 + int64_t(seq) * 202;
    summarize(clean, times);
    CHECK(!clean.stable() && clean.delayGrowthMs > 2);
    // A host that misses a few sends (a stalled send buffer) is measured as loss,
    // not a failed step; one that cannot send 98% is overloaded.
    clean = sample(500000, true);
    clean.sent = clean.received = 9950;
    clean.lossPercent = 0.5;
    CHECK(clean.capacityQualified() && paceQualified(clean));
    clean.sent = clean.received = 9700;
    clean.lossPercent = 3;
    CHECK(!clean.capacityQualified() && !paceQualified(clean));
    CHECK(std::string(clean.failureReason()) == "host did not send enough probe packets");
    // Loss under 2% qualifies capacity; 2% does not.
    clean = sample(500000, true);
    clean.lossPercent = clean.worstWindowLossPercent = 1.9;
    clean.received = 9810;
    CHECK(clean.capacityQualified());
    clean.lossPercent = clean.worstWindowLossPercent = 2.0;
    clean.received = 9800;
    CHECK(!clean.capacityQualified());
    CHECK(std::string(clean.failureReason()) == "packet loss exceeds the limit");
    clean = sample(500000, true);
    clean.senderMs = 2300;
    CHECK(!clean.stable());

    using namespace pyrowave::bandwidth;
    // Budget inversion for all supported packet sizes, FEC settings and FPS.
    for (int packet : {256, 1024, 1392}) for (int fec : {0, 20, 50, 100})
        for (int fps : {30, 60, 120, 240}) for (int image : {5000, 100000, 765000, 2000000}) {
            transport_t t {packet, fec, 2, 0};
            const auto wire = total_kbps(image, fps, t);
            CHECK(wire > image);
            CHECK(image_kbps(wire + 0.001, fps, t) >= image - 1);
        }
    transport_t t;
    CHECK(total_kbps(765000, 120, t) > 850000);
    CHECK(image_kbps(765000, 120, t) < 700000);

    namespace C = PyroWaveCalibration;
    CHECK(C::imageTarget(C::Minimum, 765000, 2000000) == 380000);
    CHECK(C::imageTarget(C::Recommended, 765000, 2000000) == 765000);
    CHECK(C::imageTarget(C::Recommended, 765000, 500000) == 500000);
    CHECK(C::imageTarget(C::Minimum, 5000, 2000000) == 5000);
    CHECK(C::wireTarget(C::Moderate, 950000) == 570000);
    CHECK(C::wireTarget(C::Maximum, 950000) == 950000);
    // Moderate aims for 60% of the link, never below Recommended, and steps
    // down to what calibration confirmed. On the 2.5 Gbps dock path a 1135 Mbps
    // budget gave 680 Mbps (below Recommended's ~850) when 60% applied to it.
    CHECK(C::wireTarget(C::Moderate, 1135000, 2500000, 850000) == 1135000);
    CHECK(C::wireTarget(C::Moderate, 3000000, 2500000, 850000) == 1500000);
    CHECK(C::wireTarget(C::Moderate, 3000000, 1000000, 850000) == 850000);
    CHECK(C::wireTarget(C::Moderate, 700000, 2500000, 850000) == 700000);
    CHECK(C::wireTarget(C::Maximum, 1135000, 2500000, 850000) == 1135000);
    for (int budget : {200000, 700000, 1135000, 2000000, 3000000})
        for (int link : {0, 1000000, 2500000, 10000000}) for (int recommended : {300000, 850000}) {
            const int moderate = C::wireTarget(C::Moderate, budget, link, recommended);
            CHECK(moderate <= budget);
            CHECK(moderate >= (std::min)(budget, recommended));
            CHECK(moderate <= C::wireTarget(C::Maximum, budget, link, recommended));
        }
    CHECK(C::imageQuality(C::Recommended, 500000, 765000) == C::ReducedQuality);
    CHECK(C::imageQuality(C::Recommended, 765000, 765000) == C::MeetsTarget);
    CHECK(C::imageQuality(C::Recommended, 900000, 765000) == C::MeetsTarget);
    CHECK(C::imageQuality(C::Recommended, 380000, 765000) == C::ReducedQuality);
    CHECK(C::imageQuality(C::Recommended, 375000, 765000) == C::BelowMinimum);
    CHECK(C::imageQuality(C::Minimum, 380000, 765000) == C::MeetsTarget);
    CHECK(C::imageQuality(C::Minimum, 375000, 765000) == C::BelowMinimum);
    CHECK(C::imageQuality(C::Minimum, 0, 5000) == C::BelowMinimum);
    // The measured wire rate is not image quality: reserve FEC and headers,
    // then apply Moderate's allowance before grading against the guide.
    CHECK(C::imageQuality(C::Recommended, C::imageCapacity(500000, 120, t), 765000) == C::ReducedQuality);
    CHECK(C::imageQuality(C::Minimum, C::imageCapacity(500000, 120, t), 765000) == C::MeetsTarget);
    CHECK(C::imageQuality(C::Recommended, C::imageCapacity(765000, 120, t), 765000) != C::MeetsTarget);
    CHECK(C::imageQuality(C::Moderate, C::imageCapacity(C::wireTarget(C::Moderate, 950000), 120, t), 765000) == C::ReducedQuality);
    for (int fps : {30, 60, 120, 240}) for (int guide : {5000, 100000, 765000, 2000000}) {
        const int ceiling = C::qualityProbeCeiling(guide, fps, t);
        const int confirmed = C::roundDown(ceiling * 0.95);
        CHECK(C::roundUp(total_kbps(guide, fps, t)) <= confirmed);
    }
    // Moderate's 60% allowance includes FEC and headers rather than adding
    // overhead after spending 60% on the image.
    for (int packet : {256, 1024, 1392}) for (int fec : {0, 20, 50, 100})
        for (int fps : {30, 60, 120, 240}) for (int wire : {5000, 500000, 950000, 3000000}) {
            transport_t transport {packet, fec, 2, 0};
            const int budget = C::wireTarget(C::Moderate, wire);
            const int image = C::imageCapacity(budget, fps, transport);
            if (image >= 5000) {
                CHECK(C::roundUp(total_kbps(image, fps, transport)) <= budget);
                CHECK(image * 125.0 / fps <= (fec ? 3000 : 4000) * (packet - 16) - 8);
            }
        }

    struct Cost { bool ok; double meanMs; double frameMs; bool overloaded; };
    std::vector<int> full, quick;
    const auto keepsUp = [](const Cost& c) { return c.ok && !c.overloaded && c.frameMs <= 8; };
    const auto device = [&](int requested, int guide, int floor, double fixed, double variable) {
        full.clear(); quick.clear();
        const auto cost = [&](int rate) {
            const double mean = fixed + variable * rate / 1000000;
            return Cost {true, mean, mean * 1.1, mean >= 8};
        };
        return C::searchDevice(requested, guide, floor,
            [&](int rate) { full.push_back(rate); return cost(rate); },
            [&](int rate) { quick.push_back(rate); return cost(rate); },
            keepsUp, [] { return false; });
    };
    // Healthy devices stop after one complete measurement at any target.
    for (int rate : {380000, 765000, 1500000, 2500000}) {
        const auto selected = device(rate, 765000, 100000, 2, 0);
        CHECK(selected.imageKbps == rate && keepsUp(selected.cost));
        CHECK(!selected.deviceLimited && full.size() == 1 && quick.empty());
    }
    // Noisy tails or fixed format overload must not buy a lucky pass at a
    // lower bitrate without the original 10% measurable-cost improvement.
    auto selected = device(2500000, 765000, 100000, 8.5, 0);
    CHECK(selected.imageKbps == 2500000 && !keepsUp(selected.cost));
    CHECK(full.size() == 2 && quick.size() == 1);
    selected = device(765000, 765000, 100000, 7.5, 0);
    CHECK(!selected.deviceLimited && !keepsUp(selected.cost));
    // A bitrate-sensitive device is refined to within 1/64 of its bracket,
    // including rates above the quality regression's upper end.
    selected = device(2500000, 765000, 100000, 2, 4);
    const int boundary = C::roundDown((8 / 1.1 - 2) / 4 * 1000000);
    CHECK(keepsUp(selected.cost) && selected.deviceLimited);
    CHECK(selected.imageKbps <= boundary && boundary - selected.imageKbps <= 30000);
    CHECK(full.size() <= 8);
    CHECK(std::find(full.begin(), full.end(), selected.imageKbps) != full.end());
    // Recover below the recommendation only with an effective floor probe.
    selected = device(765000, 765000, 100000, 2, 10);
    CHECK(keepsUp(selected.cost) && selected.deviceLimited && selected.imageKbps < 765000);
    CHECK(quick.size() == 1 && full.size() <= 8);
    CHECK(!device(100000, 765000, 100000, 9, 0).deviceLimited);
    // A runtime failure or cancellation cannot become a passing result.
    const auto invalid = C::searchDevice(765000, 765000, 100000,
        [](int) { return Cost {false, 0, 0, false}; },
        [](int) { std::abort(); return Cost {}; }, keepsUp, [] { return false; });
    CHECK(!invalid.cost.ok);
    const auto cancelled = C::searchDevice(765000, 765000, 100000,
        [](int) { return Cost {true, 9, 10, true}; },
        [](int) { std::abort(); return Cost {}; }, keepsUp, [] { return true; });
    CHECK(!keepsUp(cancelled.cost));
    // Frame pacing: highest loss-free pace, else the pace that lost least.
    const auto lossy = [](int kbps, double loss) {
        auto measured = sample(kbps, true);
        measured.lossPercent = measured.worstWindowLossPercent = loss;
        measured.received = uint32_t(measured.expected * (1 - loss / 100));
        measured.delayP99Ms = 30; // Bursty delivery never grades a pace probe.
        return measured;
    };
    std::vector<int> paces;
    const auto paceRun = [&](int link, int floor, auto lossAt) {
        paces.clear();
        return searchPace(link, floor, [&](int pace) { paces.push_back(pace); return lossy(pace, lossAt(pace)); },
                          [] { return false; });
    };
    // A receiver that keeps up at line rate is paced at the link after two confirmations.
    auto pace = paceRun(2500000, 1200000, [](int) { return 0.0; });
    CHECK(pace.lossless && pace.paceKbps == 2500000 && paces == std::vector<int>({2500000, 2500000, 2500000}));
    // The measured dock: whole USB transfers drop above 2.2 Gbps. The bisected
    // edge keeps a 5% margin, confirmed twice, and never probes below the floor.
    pace = paceRun(2500000, 1200000, [](int p) { return p <= 2200000 ? 0.0 : 2.0; });
    CHECK(pace.lossless && pace.paceKbps == 2050000);
    CHECK(paces[paces.size() - 1] == 2050000 && paces[paces.size() - 2] == 2050000);
    CHECK(*std::min_element(paces.begin(), paces.end()) >= 1200000 && paces.size() <= 12);
    // PyroWave tolerates loss as detail: under 2% qualifies, even at the link.
    pace = paceRun(2500000, 1200000, [](int p) { return p <= 2150000 ? 0.0 : 0.12; });
    CHECK(pace.lossless && pace.paceKbps == 2500000);
    pace = paceRun(2500000, 1200000, [](int p) { return p <= 2150000 ? 0.5 : 2.5; });
    CHECK(pace.lossless && pace.paceKbps <= 2150000 * 0.95 && pace.paceKbps >= 2000000);
    // Exactly 2% is not under the limit, and bunched loss fails on its own.
    pace = paceRun(2500000, 1200000, [](int) { return 2.0; });
    CHECK(!pace.lossless);
    pace = searchPace(2500000, 1200000, [&](int p) {
        auto measured = lossy(p, 0.5);
        measured.worstWindowLossPercent = p > 1800000 ? 6.0 : 1.0;
        return measured;
    }, [] { return false; });
    CHECK(pace.lossless && pace.paceKbps <= 1800000 * 0.95);
    // Nothing within the limit: choose the least loss, the faster pace on ties.
    pace = paceRun(2500000, 1200000, [](int p) { return 1.0 + p / 1000000.0; });
    CHECK(!pace.lossless && pace.paceKbps == 1200000);
    pace = paceRun(2500000, 1200000, [](int) { return 3.0; });
    CHECK(!pace.lossless && pace.paceKbps == 2500000);
    // A frame that needs most of the link keeps the link: the floor never exceeds it.
    pace = paceRun(1000000, 1500000, [](int) { return 2.5; });
    CHECK(!pace.lossless && pace.paceKbps == 1000000 && paces.size() == 1);
    // A lucky single pass is not enough: failed confirmation steps down 10%.
    int calls = 0;
    pace = searchPace(2500000, 1200000, [&](int p) {
        ++calls;
        return lossy(p, p <= 1800000 || (p <= 2000000 && calls % 2) ? 0.0 : 3.0);
    }, [] { return false; });
    CHECK(pace.lossless && pace.paceKbps <= 1800000 && pace.paceKbps >= 1200000);
    // Cancellation and unusable links determine nothing.
    bool stopPace = false;
    pace = searchPace(2500000, 1200000, [&](int p) { stopPace = true; return lossy(p, 0); }, [&] { return stopPace; });
    CHECK(pace.paceKbps == 0);
    CHECK(searchPace(10000, 5000, [&](int p) { return lossy(p, 0); }, [] { return false; }).paceKbps == 0);
    std::puts("PyroWave link search, loss, delay and FEC budget tests passed");
}
