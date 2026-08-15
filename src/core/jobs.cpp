#include "core/jobs.hpp"

#include <algorithm>
#include <chrono>

namespace skein {
namespace {
thread_local int tlsWorkerId = -1;
}

JobSystem::JobSystem(int workerCount) {
    int n = workerCount;
    if (n < 0) {
        unsigned hw = std::thread::hardware_concurrency();
        n = hw > 1 ? static_cast<int>(hw) - 1 : 0;
    }
    queues_.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) queues_.push_back(std::make_unique<Queue>());
    threads_.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) threads_.emplace_back([this, i] { workerLoop(i); });
}

JobSystem::~JobSystem() {
    running_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(sleepMutex_);
    }
    sleepCv_.notify_all();
    for (auto& t : threads_)
        if (t.joinable()) t.join();
}

JobSystem& JobSystem::global() {
    static JobSystem js;
    return js;
}

bool JobSystem::tryPopLocal(int id, Task& out) {
    if (id < 0 || id >= workerCount()) return false;
    Queue& q = *queues_[static_cast<size_t>(id)];
    std::lock_guard<std::mutex> lk(q.mutex);
    if (q.tasks.empty()) return false;
    out = std::move(q.tasks.back());
    q.tasks.pop_back();
    return true;
}

bool JobSystem::trySteal(int self, Task& out) {
    int n = workerCount();
    for (int i = 0; i < n; ++i) {
        int victim = (self + 1 + i) % n;
        if (victim == self) continue;
        Queue& q = *queues_[static_cast<size_t>(victim)];
        std::lock_guard<std::mutex> lk(q.mutex);
        if (q.tasks.empty()) continue;
        out = std::move(q.tasks.front());
        q.tasks.pop_front();
        steals_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

void JobSystem::submit(int preferred, Task task) {
    int n = workerCount();
    if (n == 0) {
        task();
        return;
    }
    int target = preferred >= 0 ? preferred % n
                                : static_cast<int>(submitCursor_.fetch_add(1, std::memory_order_relaxed) % n);
    pendingTasks_.fetch_add(1, std::memory_order_release);
    Queue& q = *queues_[static_cast<size_t>(target)];
    {
        std::lock_guard<std::mutex> lk(q.mutex);
        q.tasks.push_back(std::move(task));
    }
}

void JobSystem::wake(int n) {
    if (queues_.empty()) return;
    {
        std::lock_guard<std::mutex> lk(sleepMutex_);
    }
    if (n >= workerCount())
        sleepCv_.notify_all();
    else
        for (int i = 0; i < n; ++i) sleepCv_.notify_one();
}

bool JobSystem::helpOnce(int self) {
    // Looking for work means locking this worker's queue and then every other
    // worker's in turn. Idle threads do that in a tight loop, and eight of them
    // trading eight cache lines slows down the threads that did find work. One
    // relaxed load answers for all of it when there is nothing to find.
    if (pendingTasks_.load(std::memory_order_acquire) == 0) return false;
    Task t;
    if (tryPopLocal(self, t) || trySteal(self, t)) {
        t();
        pendingTasks_.fetch_sub(1, std::memory_order_release);
        return true;
    }
    return false;
}

void JobSystem::workerLoop(int id) {
    tlsWorkerId = id;
    while (running_.load(std::memory_order_acquire)) {
        if (helpOnce(id)) continue;
        std::unique_lock<std::mutex> lk(sleepMutex_);
        if (!running_.load(std::memory_order_acquire)) break;
        if (pendingTasks_.load(std::memory_order_acquire) == 0)
            sleepCv_.wait_for(lk, std::chrono::milliseconds(2));
    }
    tlsWorkerId = -1;
}

void JobSystem::parallelFor(size_t count, size_t grain, const std::function<void(size_t, size_t)>& body) {
    if (count == 0) return;
    if (grain == 0) grain = 1;
    int n = workerCount();
    size_t chunks = (count + grain - 1) / grain;
    if (n == 0 || chunks == 1) {
        body(0, count);
        return;
    }

    // One task per chunk means one heap-allocated std::function and one queue
    // lock per chunk, which is most of the cost of a short parallel pass. The
    // chunks are handed out through an atomic cursor instead, so the queue only
    // ever sees one task per worker and the hand-out still balances itself.
    // `remaining` counts drains still running, not chunks left, so a worker
    // never touches this stack frame after the count it is waited on reaches
    // zero.
    struct Range {
        const std::function<void(size_t, size_t)>& body;
        std::atomic<size_t> cursor{0};
        std::atomic<size_t> remaining{0};
        size_t count = 0, grain = 0, chunks = 0;

        void drain() {
            for (;;) {
                size_t c = cursor.fetch_add(1, std::memory_order_relaxed);
                if (c >= chunks) break;
                size_t begin = c * grain;
                body(begin, std::min(begin + grain, count));
            }
            remaining.fetch_sub(1, std::memory_order_release);
        }
    };
    Range range{body};
    range.count = count;
    range.grain = grain;
    range.chunks = chunks;

    int self = tlsWorkerId;
    size_t helpers = std::min<size_t>(chunks - 1, static_cast<size_t>(n));
    range.remaining.store(helpers + 1, std::memory_order_relaxed);
    for (size_t i = 0; i < helpers; ++i) submit(-1, [&range] { range.drain(); });
    wake(static_cast<int>(helpers));
    range.drain();

    while (range.remaining.load(std::memory_order_acquire) != 0) {
        if (!helpOnce(self)) std::this_thread::yield();
    }
}

void JobSystem::runAll(const std::vector<Task>& tasks) {
    if (tasks.empty()) return;
    int n = workerCount();
    if (n == 0) {
        for (const auto& t : tasks) t();
        return;
    }
    std::atomic<size_t> remaining{tasks.size()};
    int self = tlsWorkerId;
    for (const auto& t : tasks) {
        submit(-1, [&t, &remaining] {
            t();
            remaining.fetch_sub(1, std::memory_order_release);
        });
    }
    wake(static_cast<int>(std::min(tasks.size(), static_cast<size_t>(n))));
    while (remaining.load(std::memory_order_acquire) != 0) {
        if (!helpOnce(self)) std::this_thread::yield();
    }
}

}  // namespace skein
