#pragma once
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

struct TestCase {
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& testRegistry();

struct TestRegistrar {
    TestRegistrar(const char* name, void (*fn)()) { testRegistry().push_back({name, fn}); }
};

#define TEST(name)                                        \
    static void name();                                   \
    static TestRegistrar registrar_##name(#name, &name);  \
    static void name()

struct TestFailure : std::runtime_error {
    using std::runtime_error::runtime_error;
};

inline void failAt(const char* file, int line, const std::string& what) {
    throw TestFailure(std::string(file) + ":" + std::to_string(line) + "  " + what);
}

#define CHECK(cond)                                              \
    do {                                                         \
        if (!(cond)) failAt(__FILE__, __LINE__, "CHECK(" #cond ")"); \
    } while (0)

#define CHECK_EQ(a, b)                                                                    \
    do {                                                                                  \
        auto lhs_ = (a);                                                                  \
        auto rhs_ = (b);                                                                  \
        if (!(lhs_ == rhs_))                                                              \
            failAt(__FILE__, __LINE__, std::string(#a " == " #b " (") + std::to_string(lhs_) + \
                                            " vs " + std::to_string(rhs_) + ")");         \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                              \
    do {                                                                                   \
        double lhs_ = static_cast<double>(a);                                              \
        double rhs_ = static_cast<double>(b);                                              \
        if (std::fabs(lhs_ - rhs_) > (eps))                                                \
            failAt(__FILE__, __LINE__, std::string(#a " ~= " #b " (") + std::to_string(lhs_) + \
                                            " vs " + std::to_string(rhs_) + ")");          \
    } while (0)
