/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rmt.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"

#include "esp32_gpio.h"
#include "esp32_rmt.h"
#include "esp32_irq.h"
#include "esp32_clockconfig.h"

#include "hardware/esp32_dport.h"
#include "hardware/esp32_gpio_sigmap.h"

#ifdef CONFIG_ESP32_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RMT methods */

static void rmt_reset(struct rmt_dev_s *dev);
static int rmt_setup(struct rmt_dev_s *dev);
IRAM_ATTR static int rmt_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct rmt_dev_s g_rmt_dev =
{
  .channel = 0,
  .periph  = ESP32_PERIPH_RMT,
  .irq     = ESP32_IRQ_RMT,
  .cpu     = 0,
  .cpuint  = -ENOMEM,
  .lock    = SP_UNLOCKED
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmt_reset
 *
 * Description:
 *   Reset the RMT device.  Called early to initialize the hardware. This
 *   function is called, before esp32_rmt_setup().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" RMT driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void rmt_reset(struct rmt_dev_s *dev)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 1);
  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 0);
  putreg32(0xffffffff, RMT_INT_CLR_REG); /* Clear any spurious IRQ Flag */

  spin_unlock_irqrestore(&dev->lock, flags);
}

/****************************************************************************
 * Name: rmt_setup
 *
 * Description:
 *   Configure the RMT. This method is called the first time that the RMT
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching RMT interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" RMT driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int rmt_setup(struct rmt_dev_s *dev)
{
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&dev->lock);

  if (dev->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(dev->cpuint);
    }

  dev->cpu = up_cpu_index();
  dev->cpuint = esp32_setup_irq(dev->cpu, dev->periph,
                                1, ESP32_CPUINT_LEVEL);
  if (dev->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = dev->cpuint;
      spin_unlock_irqrestore(&dev->lock, flags);

      return ret;
    }

  ret = irq_attach(dev->irq, rmt_interrupt, dev);

  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32_teardown_irq(dev->cpu, dev->periph, dev->cpuint);
      dev->cpuint = -ENOMEM;
      spin_unlock_irqrestore(&dev->lock, flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the RMT device. */

  up_enable_irq(dev->irq);

  spin_unlock_irqrestore(&dev->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rmt_interrupt
 *
 * Description:
 *   RMT TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - The pointer to driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

IRAM_ATTR static int rmt_interrupt(int irq, void *context, void *arg)
{
  struct rmt_dev_s *dev = (struct rmt_dev_s *)arg;
  uint32_t regval = getreg32(RMT_INT_ST_REG);
  uint32_t total_pixels;
  uint32_t current_pixel;
  uint32_t dst_mem;
  uint32_t used_words;
  uint8_t *pixel_ptr;

  if (regval & RMT_CHN_TX_END_INT_ST(dev->channel))
    {
      putreg32(RMT_CHN_TX_END_INT_CLR(dev->channel), RMT_INT_CLR_REG);
      modifyreg32(RMT_INT_ENA_REG, RMT_CHN_TX_END_INT_ENA(dev->channel), 0);
      nxsem_post(&dev->tx_sem);
    }
  else if (regval & RMT_CHN_TX_THR_EVENT_INT_ST(dev->channel))
    {
      putreg32(RMT_CHN_TX_THR_EVENT_INT_CLR(dev->channel), RMT_INT_CLR_REG);
      total_pixels = dev->data_len;
      current_pixel = dev->next_pixel_to_load;
      dst_mem = dev->start_address;
      if (dev->frame++ == 1)
        {
          dst_mem += dev->reload_thresh * 4;
        }

      if (dev->frame > 1)
        {
          dev->frame = 0;
        }

      used_words = 0;
      pixel_ptr = dev->data_ptr + current_pixel;

      while (used_words < dev->reload_thresh)
        {
          if (current_pixel < total_pixels)
            {
              register uint8_t pixel = (*pixel_ptr++);
              for (register uint32_t i = 0; i < 8; i++)
                {
                  if (pixel & 0x80)
                    {
                      putreg32(dev->logic_one, dst_mem);
                    }
                  else
                    {
                      putreg32(dev->logic_zero, dst_mem);
                    }

                  pixel <<= 1;
                  dst_mem += 4;
                }

              used_words += 8;
              current_pixel++;
            }
          else
            {
              modifyreg32(RMT_INT_ENA_REG, 0,
                RMT_CHN_TX_END_INT_ENA(dev->channel));
              putreg32(0, dst_mem);
              break;
            }
        }

      dev->next_pixel_to_load = current_pixel;
    }
  else
    {
      /* Perhaps an error, have no info on that */

      putreg32(regval, RMT_INT_CLR_REG);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

struct rmt_dev_s *esp32_rmtinitialize(int output_pin, int channel)
{
  struct rmt_dev_s *rmtdev = &g_rmt_dev;
  irqstate_t flags;

  rmtdev->channel = channel;

  flags = spin_lock_irqsave(&rmtdev->lock);

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_RMT_CLK_EN);
  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 0);

  /* Configure RMT GPIO pin */

  esp32_gpio_matrix_out(output_pin, RMT_SIG_OUT0_IDX + channel, 0, 0);
  esp32_configgpio(output_pin, OUTPUT_FUNCTION_1);

  spin_unlock_irqrestore(&rmtdev->lock, flags);

  nxsem_init(&rmtdev->tx_sem, 0, 1);
  rmt_reset(rmtdev);
  rmt_setup(rmtdev);

  return rmtdev;
}
#endif
