#ifndef CUBS2_TEST_ZEPHYR_KERNEL_H_
#define CUBS2_TEST_ZEPHYR_KERNEL_H_

struct k_mutex {
  int unused;
};

#define K_FOREVER 0

static inline void k_mutex_init(struct k_mutex *mutex) { (void)mutex; }
static inline void k_mutex_lock(struct k_mutex *mutex, int timeout) {
  (void)mutex;
  (void)timeout;
}
static inline void k_mutex_unlock(struct k_mutex *mutex) { (void)mutex; }

#endif
