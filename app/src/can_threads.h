#ifndef CAN_THREADS
#define CAN_THREADS

#include <zephyr/kernel.h>
#include <zephyr/device.h>

void can_init(const struct device * can_dev);
void can_threads_init(void);

#endif /* CAN_THREADS */
