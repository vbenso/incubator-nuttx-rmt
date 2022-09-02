/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rmt.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <semaphore.h>
#include <nuttx/spinlock.h>
#include "hardware/esp32_rmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32_RMT)

struct rmt_dev_s
{
  /* Device configuration */

  uint8_t channel;
  uint8_t periph;     /* Peripheral ID */
  uint8_t irq;        /* IRQ associated with this RMT */
  uint8_t cpu;        /* CPU ID */
  int cpuint;         /* CPU interrupt assigned to this RMT */
  uint32_t              available_words;
  uint32_t              start_address;
  uint32_t              reload_thresh;
  uint32_t frame;
  size_t data_len;
  const uint8_t *data_ptr;
  uint32_t next_pixel_to_load;
  sem_t tx_sem;
  spinlock_t lock;
  uint32_t logic_one;  /* Pulse shape for High Logic */
  uint32_t logic_zero; /* Pulse shape for Low Logic */
};

/****************************************************************************
 * Name: esp32_rmtinitialize
 *
 * Description:
 *   Initialize the selected RMT device
 *
 * Input Parameters:
 *   output_pin - the pin used for output
 *   channel    - the RMT's channel that will be used
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct rmt_dev_s *esp32_rmtinitialize(int output_pin, int channel);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H */
