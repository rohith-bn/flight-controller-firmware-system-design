/**
 * @file test_framework.h
 * @brief Lightweight Host-Based Unit Testing Micro-Framework for QFC-1
 * @author Rohith B Narasimhamurthy
 * @standard ISO 26262-6 Software Unit Verification
 * 
 * Provides zero-dependency, self-contained test assertion macros, floating-point
 * comparison with configurable epsilon tolerances, and test execution counters.
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Global test statistics */
static int g_test_count = 0;
static int g_test_passed = 0;
static int g_test_failed = 0;

/* ANSI Terminal Colors */
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_CYAN    "\x1b[36m"

#define TEST_SUITE_START(suite_name) \
    do { \
        printf(ANSI_COLOR_CYAN "\n==================================================\n" ANSI_COLOR_RESET); \
        printf(ANSI_COLOR_CYAN "  TEST SUITE: %s\n" ANSI_COLOR_RESET, suite_name); \
        printf(ANSI_COLOR_CYAN "==================================================\n" ANSI_COLOR_RESET); \
    } while(0)

#define TEST_CASE(case_name) \
    printf(ANSI_COLOR_YELLOW "\n[*] Testing: %s\n" ANSI_COLOR_RESET, case_name)

#define TEST_ASSERT(cond, msg) \
    do { \
        g_test_count++; \
        if (cond) { \
            g_test_passed++; \
            printf("  " ANSI_COLOR_GREEN "[PASS]" ANSI_COLOR_RESET " %s\n", msg); \
        } else { \
            g_test_failed++; \
            printf("  " ANSI_COLOR_RED "[FAIL]" ANSI_COLOR_RESET " %s (%s:%d)\n", msg, __FILE__, __LINE__); \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_NEAR(actual, expected, tol, msg) \
    do { \
        g_test_count++; \
        float diff = fabsf((float)(actual) - (float)(expected)); \
        if (diff <= (float)(tol)) { \
            g_test_passed++; \
            printf("  " ANSI_COLOR_GREEN "[PASS]" ANSI_COLOR_RESET " %s (act: %8.4f, exp: %8.4f)\n", msg, (double)(actual), (double)(expected)); \
        } else { \
            g_test_failed++; \
            printf("  " ANSI_COLOR_RED "[FAIL]" ANSI_COLOR_RESET " %s (%s:%d) -> act: %8.4f, exp: %8.4f, diff: %8.4f > tol: %8.4f\n", \
                   msg, __FILE__, __LINE__, (double)(actual), (double)(expected), (double)diff, (double)(tol)); \
        } \
    } while(0)

#define TEST_ASSERT_INT_EQUAL(actual, expected, msg) \
    do { \
        g_test_count++; \
        if ((int32_t)(actual) == (int32_t)(expected)) { \
            g_test_passed++; \
            printf("  " ANSI_COLOR_GREEN "[PASS]" ANSI_COLOR_RESET " %s (val: %d)\n", msg, (int)(actual)); \
        } else { \
            g_test_failed++; \
            printf("  " ANSI_COLOR_RED "[FAIL]" ANSI_COLOR_RESET " %s (%s:%d) -> act: %d != exp: %d\n", \
                   msg, __FILE__, __LINE__, (int)(actual), (int)(expected)); \
        } \
    } while(0)

#define TEST_REPORT() \
    do { \
        printf(ANSI_COLOR_CYAN "\n--------------------------------------------------\n" ANSI_COLOR_RESET); \
        printf("Results: Total: %d | " ANSI_COLOR_GREEN "Passed: %d" ANSI_COLOR_RESET " | " \
               ANSI_COLOR_RED "Failed: %d\n" ANSI_COLOR_RESET, g_test_count, g_test_passed, g_test_failed); \
        if (g_test_failed == 0) { \
            printf(ANSI_COLOR_GREEN "[SUCCESS] ALL SUITE TESTS PASSED 100%%\n" ANSI_COLOR_RESET); \
        } else { \
            printf(ANSI_COLOR_RED "[FAILURE] %d TEST(S) FAILED!\n" ANSI_COLOR_RESET, g_test_failed); \
        } \
        printf(ANSI_COLOR_CYAN "==================================================\n" ANSI_COLOR_RESET); \
    } while(0)

#define TEST_EXIT_CODE() (g_test_failed == 0 ? 0 : 1)

#endif /* TEST_FRAMEWORK_H */
