#include "core/profiler.hpp"

#include <algorithm>
#include <cstdio>

namespace skein {
namespace {

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t i = static_cast<size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
    return v[std::min(i, v.size() - 1)];
}

}  // namespace

Profiler& Profiler::instance() {
    static Profiler p;
    return p;
}

int Profiler::zone(const char* name) {
    Profiler& self = instance();
    std::lock_guard<std::mutex> lk(self.mutex_);
    for (size_t i = 0; i < self.zoneCount_; ++i)
        if (self.zones_[i].name == name) return static_cast<int>(i);
    if (self.zoneCount_ >= MAX_ZONES) return -1;
    size_t id = self.zoneCount_++;
    self.zones_[id].name = name;
    return static_cast<int>(id);
}

void Profiler::addSample(int handle, uint64_t nanos) {
    if (handle < 0 || static_cast<size_t>(handle) >= MAX_ZONES) return;
    zones_[static_cast<size_t>(handle)].nanos.fetch_add(nanos, std::memory_order_relaxed);
    zones_[static_cast<size_t>(handle)].calls.fetch_add(1, std::memory_order_relaxed);
}

void Profiler::setCounter(const char* name, double value) {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& c : counters_)
        if (c.first == name) {
            c.second = value;
            return;
        }
    counters_.emplace_back(name, value);
}

void Profiler::beginFrame() { frameStart_ = Clock::now(); }

void Profiler::endFrame() {
    frameMs_ = millisSince(frameStart_);
    std::lock_guard<std::mutex> lk(mutex_);
    for (size_t i = 0; i < zoneCount_; ++i) {
        Zone& z = zones_[i];
        double ms = static_cast<double>(z.nanos.exchange(0, std::memory_order_relaxed)) * 1e-6;
        z.lastCalls = z.calls.exchange(0, std::memory_order_relaxed);
        z.lastMs = ms;
        z.history[historyPos_] = ms;
    }
    frameHistory_[historyPos_] = frameMs_;
    historyPos_ = (historyPos_ + 1) % HISTORY;
    historyFill_ = std::min(historyFill_ + 1, HISTORY);
    ++frameIndex_;
}

double Profiler::avgFrameMs() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (historyFill_ == 0) return 0;
    double sum = 0;
    for (size_t i = 0; i < historyFill_; ++i) sum += frameHistory_[i];
    return sum / static_cast<double>(historyFill_);
}

double Profiler::p95FrameMs() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return percentile({frameHistory_.begin(), frameHistory_.begin() + static_cast<long>(historyFill_)}, 0.95);
}

std::vector<ZoneStats> Profiler::stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<ZoneStats> out;
    out.reserve(zoneCount_);
    for (size_t i = 0; i < zoneCount_; ++i) {
        const Zone& z = zones_[i];
        std::vector<double> h(z.history.begin(), z.history.begin() + static_cast<long>(historyFill_));
        ZoneStats s;
        s.name = z.name;
        s.lastMs = z.lastMs;
        s.calls = z.lastCalls;
        if (!h.empty()) {
            double sum = 0;
            s.minMs = h[0];
            s.maxMs = h[0];
            for (double v : h) {
                sum += v;
                s.minMs = std::min(s.minMs, v);
                s.maxMs = std::max(s.maxMs, v);
            }
            s.avgMs = sum / static_cast<double>(h.size());
            s.p95Ms = percentile(h, 0.95);
        }
        out.push_back(std::move(s));
    }
    return out;
}

std::vector<std::pair<std::string, double>> Profiler::counters() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return counters_;
}

std::string Profiler::report() const {
    auto zs = stats();
    std::sort(zs.begin(), zs.end(), [](const ZoneStats& a, const ZoneStats& b) { return a.avgMs > b.avgMs; });
    std::string out;
    char line[256];
    std::snprintf(line, sizeof(line), "frame  avg %.3f ms  p95 %.3f ms  (%.1f fps)\n", avgFrameMs(),
                  p95FrameMs(), avgFrameMs() > 0 ? 1000.0 / avgFrameMs() : 0.0);
    out += line;
    std::snprintf(line, sizeof(line), "%-24s %9s %9s %9s %7s\n", "zone", "avg ms", "p95 ms", "max ms", "calls");
    out += line;
    for (const auto& s : zs) {
        std::snprintf(line, sizeof(line), "%-24s %9.3f %9.3f %9.3f %7u\n", s.name.c_str(), s.avgMs, s.p95Ms,
                      s.maxMs, s.calls);
        out += line;
    }
    for (const auto& c : counters()) {
        std::snprintf(line, sizeof(line), "%-24s %9.0f\n", c.first.c_str(), c.second);
        out += line;
    }
    return out;
}

void Profiler::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    for (size_t i = 0; i < zoneCount_; ++i) {
        zones_[i].nanos.store(0);
        zones_[i].calls.store(0);
        zones_[i].history.fill(0);
        zones_[i].lastMs = 0;
        zones_[i].lastCalls = 0;
    }
    frameHistory_.fill(0);
    historyFill_ = 0;
    historyPos_ = 0;
    frameIndex_ = 0;
}

}  // namespace skein
