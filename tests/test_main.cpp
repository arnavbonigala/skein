#include "test.hpp"

#include <cstring>

std::vector<TestCase>& testRegistry() {
    static std::vector<TestCase> cases;
    return cases;
}

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int passed = 0;
    std::vector<std::string> failures;

    for (const TestCase& t : testRegistry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        try {
            t.fn();
            ++passed;
            std::printf("  ok   %s\n", t.name);
        } catch (const std::exception& e) {
            failures.push_back(std::string(t.name) + "\n       " + e.what());
            std::printf("  FAIL %s\n       %s\n", t.name, e.what());
        }
    }

    std::printf("\n%d passed, %zu failed\n", passed, failures.size());
    return failures.empty() ? 0 : 1;
}
