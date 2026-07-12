#ifndef CUBS2_TEST_ZEPHYR_UTIL_H_
#define CUBS2_TEST_ZEPHYR_UTIL_H_

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define ARG_UNUSED(argument) (void)(argument)
#define BUILD_ASSERT(condition) _Static_assert(condition, #condition)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define CLAMP(value, low, high) MIN((high), ((value) < (low) ? (low) : (value)))

#endif
