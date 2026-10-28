// Minimal standalone test harness for the libstd suites.
//
// It uses only C headers and compiler language features, so it can be compiled next to the canonical
// ETLStd headers (-ffreestanding -nostdinc++) without the host C++ standard library that Catch loads.
//
//   TEST_CASE("title")      registers the function body that follows (extra arguments, such as Catch tags, are ignored)
//   CHECK(expr)             records a failure with file and line when expr is false
//   CHECK_EQ(a, b)          same for a == b, printing both values when they are convertible to long long
//   CHECK_THROWS_AS(e, T)   expects the evaluation of e to throw an exception of type T
//
// main() lives in main.cpp and returns non-zero if any check failed.
#pragma once

#include <stdio.h>
#include <stdlib.h>

namespace etl_harness {

struct TestCase {
    const char *title;
    const char *file;
    int line;
    void (*function)();
    TestCase *next;
};

inline TestCase *firstTest = nullptr;
inline TestCase *lastTest = nullptr;
inline int checks = 0;
inline int failures = 0;

struct Registrar {
    Registrar(TestCase &test) {
        if (lastTest)
            lastTest->next = &test;
        else
            firstTest = &test;
        lastTest = &test;
    }
};

template <typename T> void print(const char *label, const T &value) {
    if constexpr (requires { static_cast<long long>(value); })
        fprintf(stderr, "    %s = %lld\n", label, static_cast<long long>(value));
}

inline void fail(const char *file, int line, const char *message, const char *expression) {
    ++failures;
    fprintf(stderr, "%s:%d: %s(%s) failed\n", file, line, message, expression);
}

inline int run() {
    int tests = 0;
    for (TestCase *test = firstTest; test; test = test->next) {
        const int failuresBefore = failures;
        try {
            test->function();
        } catch (...) {
            fail(test->file, test->line, "unexpected exception in TEST_CASE", test->title);
        }
        ++tests;
        fprintf(stderr, "[%s] %s\n", failures == failuresBefore ? "ok" : "FAILED", test->title);
    }
    fprintf(stderr, "%d test cases, %d checks, %d failed\n", tests, checks, failures);
    return failures == 0 ? 0 : 1;
}

} // namespace etl_harness

#define ETL_HARNESS_CONCAT_(a, b) a##b
#define ETL_HARNESS_CONCAT(a, b) ETL_HARNESS_CONCAT_(a, b)

#define TEST_CASE(title, ...)                                                                                   \
    static void ETL_HARNESS_CONCAT(etl_test_, __LINE__)();                                                      \
    static etl_harness::TestCase ETL_HARNESS_CONCAT(etl_test_case_, __LINE__){title, __FILE__, __LINE__, ETL_HARNESS_CONCAT(etl_test_, __LINE__), nullptr}; \
    static etl_harness::Registrar ETL_HARNESS_CONCAT(etl_registrar_, __LINE__){ETL_HARNESS_CONCAT(etl_test_case_, __LINE__)}; \
    static void ETL_HARNESS_CONCAT(etl_test_, __LINE__)()

#define CHECK(expression)                                                                                       \
    do {                                                                                                        \
        ++etl_harness::checks;                                                                                  \
        if (!(expression))                                                                                      \
            etl_harness::fail(__FILE__, __LINE__, "CHECK", #expression);                                        \
    } while (false)

#define CHECK_EQ(left, right)                                                                                   \
    do {                                                                                                        \
        ++etl_harness::checks;                                                                                  \
        const auto &etlLeft = (left);                                                                           \
        const auto &etlRight = (right);                                                                         \
        if (!(etlLeft == etlRight)) {                                                                           \
            etl_harness::fail(__FILE__, __LINE__, "CHECK_EQ", #left ", " #right);                               \
            etl_harness::print("left ", etlLeft);                                                               \
            etl_harness::print("right", etlRight);                                                              \
        }                                                                                                       \
    } while (false)

#define CHECK_THROWS_AS(expression, exceptionType)                                                              \
    do {                                                                                                        \
        ++etl_harness::checks;                                                                                  \
        bool etlThrown = false;                                                                                 \
        try {                                                                                                   \
            (void)(expression);                                                                                 \
        } catch (const exceptionType &) {                                                                       \
            etlThrown = true;                                                                                   \
        } catch (...) {                                                                                         \
        }                                                                                                       \
        if (!etlThrown)                                                                                         \
            etl_harness::fail(__FILE__, __LINE__, "CHECK_THROWS_AS", #expression ", " #exceptionType);          \
    } while (false)
