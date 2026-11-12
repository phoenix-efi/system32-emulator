/* minitest.h -- tiny test harness (no dependencies). */
#ifndef MINITEST_H
#define MINITEST_H
#include <stdio.h>
#include <stdlib.h>

static int mt_fail, mt_checks, mt_tests;
static const char *mt_cur;

#define CHECK(cond)                                                                                          \
  do {                                                                                                       \
    mt_checks++;                                                                                             \
    if (!(cond)) { mt_fail++; fprintf(stderr, "FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__, mt_cur, #cond); } \
  } while (0)
#define CHECK_EQ(a, b)                                                                                        \
  do {                                                                                                       \
    long long _a = (long long)(a), _b = (long long)(b);                                                      \
    mt_checks++;                                                                                             \
    if (_a != _b) {                                                                                          \
      mt_fail++;                                                                                             \
      fprintf(stderr, "FAIL %s:%d [%s]: %s == %s  (0x%llX != 0x%llX)\n", __FILE__, __LINE__, mt_cur, #a, #b, \
              (unsigned long long)_a, (unsigned long long)_b);                                               \
    }                                                                                                        \
  } while (0)
#define TEST(name) static void name(void)
#define RUN(name) do { mt_cur = #name; mt_tests++; name(); } while (0)
#define MT_REPORT()                                                                                          \
  do {                                                                                                       \
    printf("%d tests, %d checks, %d failures\n", mt_tests, mt_checks, mt_fail);                              \
    return mt_fail ? 1 : 0;                                                                                  \
  } while (0)
#endif
