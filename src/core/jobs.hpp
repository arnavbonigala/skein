#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace skein {

/// Fixed-size worker pool with per-worker deques and stealing between them.
/// The thread that submits work also runs jobs while it waits, so a wait never
/// idles a core.
class JobSystem {
public:
    using Task = std::function<void()>;

    /// A negative workerCount picks hardware concurrency minus the calling
    /// thread; 0 means no worker threads at all and every job runs inline.
    explicit JobSystem(int workerCount = -1);
    ~JobSystem();

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    int workerCount() const { return static_cast<int>(queues_.size()); }
    /// Worker threads plus the submitting thread, which also executes jobs.
    int threadCount() const { return workerCount() + 1; }

    /// Splits [0, count) into chunks of at most grain and runs them concurrently.
    /// Blocks until every chunk has finished.
    void parallelFor(size_t count, size_t grain, const std::function<void(size_t, size_t)>& body);

    /// Runs independent tasks concurrently and blocks until all are done.
    void runAll(const std::vector<Task>& tasks);

    size_t stealCount() const { return steals_.load(std::memory_order_relaxed); }
    void resetCounters() { steals_.store(0, std::memory_order_relaxed); }

    static JobSystem& global();

private:
    struct Queue {
        std::mutex mutex;
        std::deque<Task> tasks;
    };

    void workerLoop(int id);
    bool tryPopLocal(int id, Task& out);
    bool trySteal(int self, Task& out);
    void submit(int preferred, Task task);
    /// Executes any available job; returns false when nothing could be found.
    bool helpOnce(int self);
    void wake(int n);

    std::vector<std::unique_ptr<Queue>> queues_;
    std::vector<std::thread> threads_;
    std::mutex sleepMutex_;
    std::condition_variable sleepCv_;
    std::atomic<bool> running_{true};
    std::atomic<size_t> pendingTasks_{0};
    std::atomic<size_t> steals_{0};
    std::atomic<uint32_t> submitCursor_{0};
};

}  // namespace skein
