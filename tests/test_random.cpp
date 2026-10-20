#include "core/random.hpp"
#include "test.hpp"

#include <algorithm>
#include <vector>

using namespace skein;

TEST(random_matches_the_pcg32_reference_sequence) {
    Random rng(42u, 54u);
    const uint32_t expected[6] = {0xa15c02b7u, 0x7b47f409u, 0xba1d3330u, 0x83d2f293u, 0xbfa4784bu, 0xcbed606eu};
    for (uint32_t e : expected) CHECK_EQ(rng.next(), e);
}

TEST(random_streams_are_reproducible_and_independent) {
    Random a(7u, 1u), b(7u, 1u), c(7u, 2u);
    int same = 0;
    for (int i = 0; i < 1000; ++i) {
        uint32_t x = a.next();
        CHECK_EQ(x, b.next());
        if (x == c.next()) ++same;
    }
    CHECK(same < 3);
}

TEST(random_ranges_stay_in_bounds_and_cover_them) {
    Random rng(1u);
    std::vector<int> hits(7, 0);
    for (int i = 0; i < 70000; ++i) {
        float u = rng.unit();
        CHECK(u >= 0.0f && u < 1.0f);
        float f = rng.uniform(-3.0f, 5.0f);
        CHECK(f >= -3.0f && f < 5.0f);
        int r = rng.range(-2, 4);
        CHECK(r >= -2 && r <= 4);
        ++hits[static_cast<size_t>(r + 2)];
    }
    for (int h : hits) CHECK(h > 9000 && h < 11000);
}

TEST(random_normal_has_unit_spread) {
    Random rng(9u);
    double sum = 0, sq = 0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        double x = rng.normal();
        sum += x;
        sq += x * x;
    }
    CHECK_NEAR(sum / n, 0.0, 0.01);
    CHECK_NEAR(sq / n, 1.0, 0.02);
}

TEST(random_works_with_standard_algorithms) {
    Random rng(3u);
    std::vector<int> v{0, 1, 2, 3, 4, 5, 6, 7};
    std::shuffle(v.begin(), v.end(), rng);
    std::sort(v.begin(), v.end());
    for (int i = 0; i < 8; ++i) CHECK_EQ(v[static_cast<size_t>(i)], i);
}
