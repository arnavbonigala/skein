#include "core/jobs.hpp"
#include "test.hpp"

#include <algorithm>
#include <atomic>
#include <numeric>

using namespace skein;

TEST(parallel_for_covers_every_index_exactly_once) {
    JobSystem js(4);
    for (size_t count : {size_t{1}, size_t{7}, size_t{64}, size_t{1000}, size_t{4097}}) {
        for (size_t grain : {size_t{1}, size_t{3}, size_t{64}, size_t{5000}}) {
            std::vector<std::atomic<int>> hits(count);
            for (auto& h : hits) h.store(0);
            js.parallelFor(count, grain, [&](size_t begin, size_t end) {
                CHECK(begin < end);
                CHECK(end <= count);
                for (size_t i = begin; i < end; ++i) hits[i].fetch_add(1);
            });
            for (size_t i = 0; i < count; ++i) CHECK_EQ(hits[i].load(), 1);
        }
    }
}

TEST(parallel_for_result_matches_serial_reduction) {
    JobSystem js(6);
    const size_t n = 500000;
    std::vector<double> values(n);
    for (size_t i = 0; i < n; ++i) values[i] = static_cast<double>(i % 97) * 0.5;

    std::atomic<double> total{0.0};
    js.parallelFor(n, 8192, [&](size_t begin, size_t end) {
        double local = 0;
        for (size_t i = begin; i < end; ++i) local += values[i];
        double expected = total.load();
        while (!total.compare_exchange_weak(expected, expected + local)) {
        }
    });
    double serial = std::accumulate(values.begin(), values.end(), 0.0);
    CHECK_NEAR(total.load(), serial, 1e-6);
}

TEST(nested_parallel_for_completes_without_deadlock) {
    JobSystem js(3);
    std::atomic<int> leaves{0};
    js.parallelFor(16, 1, [&](size_t, size_t) {
        js.parallelFor(8, 1, [&](size_t a, size_t b) { leaves.fetch_add(static_cast<int>(b - a)); });
    });
    CHECK_EQ(leaves.load(), 16 * 8);
}

TEST(run_all_executes_every_task_once) {
    JobSystem js(4);
    std::atomic<int> counter{0};
    std::vector<JobSystem::Task> tasks;
    for (int i = 0; i < 64; ++i) tasks.emplace_back([&counter] { counter.fetch_add(1); });
    js.runAll(tasks);
    CHECK_EQ(counter.load(), 64);
}

TEST(work_reaches_more_than_one_thread) {
    JobSystem js(4);
    CHECK_EQ(js.workerCount(), 4);
    std::atomic<size_t> distinct{0};
    std::vector<std::thread::id> ids(256);
    // Each chunk has to take long enough for a worker to wake and claim one:
    // chunks are handed out on demand, so an instant body is finished by the
    // calling thread alone before anyone else gets there.
    js.parallelFor(256, 1, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            volatile double spin = 0;
            for (int k = 0; k < 20000; ++k) spin += k;
            ids[i] = std::this_thread::get_id();
        }
    });
    std::vector<std::thread::id> unique(ids);
    std::sort(unique.begin(), unique.end());
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    distinct = unique.size();
    CHECK(distinct.load() > 1);
}

TEST(zero_worker_pool_still_runs_work_inline) {
    JobSystem js(0);
    CHECK_EQ(js.workerCount(), 0);
    int sum = 0;
    js.parallelFor(10, 2, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) sum += static_cast<int>(i);
    });
    CHECK_EQ(sum, 45);
}
