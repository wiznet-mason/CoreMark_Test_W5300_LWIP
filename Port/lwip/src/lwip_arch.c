/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "lwip/init.h"
#include "stdlib.h"
#include "main.h"
#include "lwip/opt.h"

#include "cmsis_os.h"

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void) {
    return 0;
}

void sys_arch_unprotect(sys_prot_t pval) {
    (void) pval;
}

/* lwip needs a millisecond time source, and the TinyUSB board support code has one available */
uint32_t sys_now(void) {
#if 0
    return HAL_GetTick();
#else
  /* Get the current tick count */
  TickType_t ticks = xTaskGetTickCount();

  /* Convert ticks to milliseconds and return */
  return (ticks * portTICK_PERIOD_MS);
#endif
}