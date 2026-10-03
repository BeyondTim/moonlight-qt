#include "../../app/streaming/video/pyrowave/pyrowavelinkpolicy.h"
#include "../../app/streaming/video/pyrowave/pyrowavebandwidth.h"
#include <cstdio>
#include <cstdlib>

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
    CHECK(run(500000, maximumKbps).requestedKbps == 2850000);
    CHECK(run(minimumKbps, maximumKbps).requestedKbps == 2850000);
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
    // A short burst is unacceptable even when aggregate loss is under 0.1%.
    for (int i = 0; i < 8; ++i) times[i] = -1;
    summarize(clean, times);
    CHECK(clean.lossPercent < 0.1 && !clean.stable());
    // Tail loss is counted against the sender's expected count.
    times.assign(clean.expected, -1);
    summarize(clean, times);
    CHECK(clean.lossPercent == 100 && !clean.stable());
    // Lossless queue growth must fail too; a constant clock offset is harmless.
    for (uint32_t seq = 0; seq < clean.expected; ++seq) times[seq] = 100000 + int64_t(seq) * 202;
    summarize(clean, times);
    CHECK(!clean.stable() && clean.delayGrowthMs > 2);
    clean = sample(500000, true);
    --clean.sent;
    CHECK(!clean.stable());
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
    std::puts("PyroWave link search, loss, delay and FEC budget tests passed");
}
