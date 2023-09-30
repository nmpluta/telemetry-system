/*
 * Copyright (c) 2022-2023 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CAN_THREADS
#define CAN_THREADS

void can_init(const struct device * can_dev);
void can_threads_init(void);

#endif /* CAN_THREADS */
