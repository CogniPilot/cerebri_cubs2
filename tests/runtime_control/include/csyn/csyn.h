#ifndef CUBS2_TEST_CSYN_H_
#define CUBS2_TEST_CSYN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*csyn_query_handler_t)(const uint8_t *request, size_t request_len,
                                     uint8_t *reply, size_t reply_capacity,
                                     size_t *reply_len, void *user);

bool csyn_zenoh_register_queryable(const char *key,
                                   csyn_query_handler_t handler, void *user);

#endif
