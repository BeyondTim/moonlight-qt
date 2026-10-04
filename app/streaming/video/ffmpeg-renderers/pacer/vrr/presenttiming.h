#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace Vrr13 {

// Rolling count of display intervals whose spacing differs from the spacing
// of the matching submissions by more than the interval tolerance. That
// difference is timing added after submission (GPU execution, compositor and
// display scheduling), which more source buffering cannot correct. It is
// reported apart from Smoothness and never feeds the buffer.
class PresentTiming {
public:
    static constexpr uint64_t BucketUs = 1000000;
    static constexpr size_t Buckets = 30;

    struct Stats {
        uint64_t intervals = 0, misses = 0, errorTotalUs = 0, lastObservedUs = 0;
        double issuePercent() const {
            return intervals ? 100.0 * double(misses) / double(intervals) : 0.0;
        }
    };

    // Only adjacent submitted frames with display feedback for both form an
    // interval. A gap (drop, missing feedback, rebase) starts a new sequence.
    void observe(uint64_t frame, uint64_t submittedUs, uint64_t presentedUs,
                 uint64_t uncertaintyUs, uint64_t observedUs, uint64_t toleranceUs)
    {
        if (m_HavePrevious && frame <= m_Previous.frame) return;
        const auto previous = m_Previous;
        const bool adjacent = m_HavePrevious && frame == previous.frame + 1 &&
            submittedUs > previous.submitted && presentedUs > previous.presented &&
            presentedUs - previous.presented < 1000000;
        m_Previous = {frame, submittedUs, presentedUs, uncertaintyUs};
        m_HavePrevious = true;
        if (!adjacent) return;

        const auto displayed = presentedUs - previous.presented;
        const auto submitted = submittedUs - previous.submitted;
        const auto raw = std::max(displayed, submitted) - std::min(displayed, submitted);
        const auto error = raw - std::min(raw, uncertaintyUs + previous.uncertainty);
        const auto tick = observedUs / BucketUs;
        auto& bucket = m_Buckets[tick % Buckets];
        if (bucket.tick != tick) bucket = Bucket{tick, 0, 0, 0};
        ++bucket.intervals;
        bucket.misses += error > toleranceUs;
        bucket.errorTotal += error;
        m_LastObserved = std::max(m_LastObserved, observedUs);
    }

    void breakSequence() { m_HavePrevious = false; }
    void reset() { *this = PresentTiming{}; }

    // The window ends at the newest observation; callers judge freshness.
    Stats stats() const
    {
        Stats s;
        s.lastObservedUs = m_LastObserved;
        const auto tick = m_LastObserved / BucketUs;
        for (const auto& b : m_Buckets) {
            if (!b.intervals || b.tick > tick || tick - b.tick >= Buckets) continue;
            s.intervals += b.intervals;
            s.misses += b.misses;
            s.errorTotalUs += b.errorTotal;
        }
        return s;
    }

private:
    struct Previous { uint64_t frame = 0, submitted = 0, presented = 0, uncertainty = 0; };
    struct Bucket { uint64_t tick = 0, intervals = 0, misses = 0, errorTotal = 0; };
    std::array<Bucket, Buckets> m_Buckets{};
    Previous m_Previous;
    uint64_t m_LastObserved = 0;
    bool m_HavePrevious = false;
};

}
