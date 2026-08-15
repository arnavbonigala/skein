#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace skein {

using Clock = std::chrono::steady_clock;

inline double millisSince(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

struct ZoneStats {
    std::string name;
    double lastMs = 0;
    double avgMs = 0;
    double minMs = 0;
    double maxMs = 0;
    double p95Ms = 0;
    uint32_t calls = 0;
};

/// Frame profiler: named zones accumulate wall time per frame from any thread,
/// then endFrame() folds them into a rolling window for percentile stats.
class Profiler {
public:
    static constexpr size_t MAX_ZONES = 64;
    static constexpr size_t HISTORY = 120;

    static Profiler& instance();

    /// Interns a zone name; call once and cache the handle.
    static int zone(const char* name);

    void addSample(int zoneHandle, uint64_t nanos);
    void setCounter(const char* name, double value);

    void beginFrame();
    void endFrame();

    double frameMs() const { return frameMs_; }
    double avgFrameMs() const;
    double p95FrameMs() const;
    uint64_t frameIndex() const { return frameIndex_; }

    std::vector<ZoneStats> stats() const;
    std::vector<std::pair<std::string, double>> counters() const;
    std::string report() const;
    void reset();

private:
    struct Zone {
        std::string name;
        std::atomic<uint64_t> nanos{0};
        std::atomic<uint32_t> calls{0};
        std::array<double, HISTORY> history{};
        uint32_t lastCalls = 0;
        double lastMs = 0;
    };

    mutable std::mutex mutex_;
    std::array<Zone, MAX_ZONES> zones_;
    size_t zoneCount_ = 0;
    std::vector<std::pair<std::string, double>> counters_;
    std::array<double, HISTORY> frameHistory_{};
    size_t historyFill_ = 0;
    size_t historyPos_ = 0;
    uint64_t frameIndex_ = 0;
    double frameMs_ = 0;
    Clock::time_point frameStart_;
};

struct ScopedZone {
    int handle;
    Clock::time_point start;
    explicit ScopedZone(int h) : handle(h), start(Clock::now()) {}
    ~ScopedZone() {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
        Profiler::instance().addSample(handle, static_cast<uint64_t>(ns));
    }
};

#define SKEIN_CONCAT_INNER(a, b) a##b
#define SKEIN_CONCAT(a, b) SKEIN_CONCAT_INNER(a, b)
#define SKEIN_PROFILE(name)                                    \
    static const int SKEIN_CONCAT(zoneHandle_, __LINE__) = ::skein::Profiler::zone(name); \
    ::skein::ScopedZone SKEIN_CONCAT(zoneScope_, __LINE__)(SKEIN_CONCAT(zoneHandle_, __LINE__))

}  // namespace skein
