/****************************************************************************
 * boards/xtensa/esp32/esp32-devkitc/src/esp32_ws2812.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include "xtensa.h"

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/leds/ws2812.h>
#include "esp32_rmt.h"

#ifdef CONFIG_WS2812

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APB_PERIOD (12.5)

#define T0H ((uint16_t)(350 / APB_PERIOD))   // ns
#define T0L ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1H ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1L ((uint16_t)(350 / APB_PERIOD))   // ns
#define RES ((uint16_t)(60000 / APB_PERIOD)) // ns

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#  ifndef CONFIG_WS2812_NON_SPI_DRIVER
/****************************************************************************
 * Name: board_ws2812_initialize
 *
 * Description:
 *   Initialize and register the WS2812 LED driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/leddrvN
 *   spino - SPI port number
 *   nleds - number of LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ws2812_initialize(int devno, int spino, uint16_t nleds)
{
  struct spi_dev_s *spi;
  char devpath[13];
  int ret;

  spi = esp32_spibus_initialize(spino);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* Register the WS2812 driver at the specified location. */

  snprintf(devpath, sizeof(devpath), "/dev/leds%d", devno);
  ret = ws2812_leds_register(devpath, spi, nleds);
  if (ret < 0)
    {
      lederr("ERROR: ws2812_leds_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}
#  else

struct ws2812_rmt_s
{
  struct rmt_dev_s *rmt_dev;
  size_t open_count;   /* Number of opens on this instance. */
};

static int ws2812_rmt_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ws2812_dev_s *dev_data = inode->i_private;
  FAR struct ws2812_rmt_s *priv = (FAR struct ws2812_rmt_s *)
                                      dev_data->private;
  int ret;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->rmt_dev->lock);
  if (priv->open_count == 0)
    {
      ledinfo("rmt_dev=%p\n", priv->rmt_dev);
      if (priv->rmt_dev)
        {
          uint32_t reg0_addr = RMT_CHNCONF0_REG(priv->rmt_dev->channel);
          uint32_t reg1_addr = RMT_CHNCONF1_REG(priv->rmt_dev->channel);
          uint32_t reg_val = 0x00;

          /* a single memory block with double buffering is enough */

          uint32_t mem_blocks = 1;
          priv->rmt_dev->available_words = 64*mem_blocks;

          uint32_t start_addr_chn = 0x3ff56800 +
                                    64 * 4 * priv->rmt_dev->channel;
          priv->rmt_dev->start_address = start_addr_chn;

          priv->rmt_dev->reload_thresh = priv->rmt_dev->available_words / 2;

          reg_val = (mem_blocks) << 24;
          uint32_t clock_divider = 1;
          reg_val |= (clock_divider);
          putreg32(reg_val, reg0_addr);

          reg_val = 0;

          /* use APB clock */

          reg_val |= RMT_REF_ALWAYS_ON_CHN;

          /* memory block in transmission mode */

          reg_val &= ~RMT_MEM_OWNER_CHN;
          putreg32(reg_val, reg1_addr);

          /* set when the buffer swapping IRQ must be generated */

          uint32_t reload_addr = RMT_CHN_TX_LIM_REG(priv->rmt_dev->channel);
          putreg32(priv->rmt_dev->reload_thresh, reload_addr);

          /* allow direct access to RMT's memory */

          modifyreg32(RMT_APB_CONF_REG, 0, BIT(0));
        }
    }

  priv->open_count += 1;

  ret = OK;

  spin_unlock_irqrestore(&priv->rmt_dev->lock, flags);

  return ret;
}

static ssize_t ws2812_rmt_write(FAR struct file *filep,
                        FAR const char *data,
                        size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ws2812_dev_s *dev_data = inode->i_private;
  FAR struct ws2812_rmt_s *priv = (FAR struct ws2812_rmt_s *)
                                      dev_data->private;
  if (data == NULL)
    {
      lederr("DATA is NULL\n");
      return 0;
    }

  nxsem_wait(&dev_data->exclsem);

  uint32_t mem = priv->rmt_dev->start_address;
  uint32_t used_words = 0;
  uint32_t current_pixel = 0;
  uint8_t *pixel_ptr = (uint8_t *)data;
  uint8_t pixel;
  uint32_t reg1_addr = RMT_CHNCONF1_REG(priv->rmt_dev->channel);
  modifyreg32(reg1_addr, 0, RMT_MEM_RD_RST_CHN);
  modifyreg32(reg1_addr, RMT_MEM_RD_RST_CHN, 0);

  while (used_words < priv->rmt_dev->available_words)
    {
      if (current_pixel < len)
        {
          pixel = *pixel_ptr++;
          for (int i = 0; i < 8; i++)
            {
              int bit = pixel & 0x80;
              if (bit)
                {
                  putreg32(priv->rmt_dev->logic_one, mem);
                }
              else
                {
                  putreg32(priv->rmt_dev->logic_zero, mem);
                }

              mem += 4;
              used_words++;
              pixel <<= 1;
            }

          current_pixel++;
        }
      else
        {
          putreg32(0, mem);
          break;
        }
    }

  if (used_words < priv->rmt_dev->available_words)
    {
      /* Small transactions won't need a the swap buffer IRQ */

      modifyreg32(
        RMT_INT_ENA_REG,
        RMT_CHN_TX_THR_EVENT_INT_ENA(priv->rmt_dev->channel),
        RMT_CHN_TX_END_INT_ENA(priv->rmt_dev->channel));
    }
  else
    {
      /* In large transactions inform the current state to the IRQ */

      priv->rmt_dev->next_pixel_to_load = current_pixel;
      priv->rmt_dev->data_ptr           = (uint8_t *) data;
      priv->rmt_dev->data_len           = len;
      priv->rmt_dev->frame              = 0;

      /* Enable memory wrap around */

      modifyreg32(RMT_APB_CONF_REG, 0, BIT(1));

      /* Enable buffering IRQ */

      modifyreg32(
        RMT_INT_ENA_REG,
        0,
        RMT_CHN_TX_END_INT_ENA(priv->rmt_dev->channel) |
        RMT_CHN_TX_THR_EVENT_INT_ENA(priv->rmt_dev->channel));
    }

  /* Start the transaction */

  modifyreg32(reg1_addr, 0, RMT_TX_START_CHN);
  nxsem_wait(&priv->rmt_dev->tx_sem);

  /* Wait for the TX lock and release it after */

  nxsem_wait(&priv->rmt_dev->tx_sem);
  nxsem_post(&priv->rmt_dev->tx_sem);
  nxsem_post(&dev_data->exclsem);

  return len;
}

/****************************************************************************
 * Name: board_ws2812_initialize
 *
 * Description:
 *   Initialize and register the WS2812 LED driver using the RMT
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/ledsN
 *   rmt_dev - Pointer to the RMT device that will be used
 *   nleds - number of LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ws2812_initialize(int devno,
                            uint16_t nleds,
                            void *dev)
{
  char devpath[13];
  int ret;

  struct ws2812_dev_s *dev_data;

  struct rmt_dev_s *rmt_dev = (struct rmt_dev_s *)dev;

  dev_data = kmm_zalloc(sizeof(struct ws2812_dev_s));

  if (dev_data == NULL)
    {
      set_errno(ENOMEM);
      return -ENOMEM;
    }

  /* Allocate struct holding out persistent data */

  struct ws2812_rmt_s *priv = kmm_zalloc(sizeof(struct ws2812_rmt_s));
  priv->rmt_dev             = rmt_dev;
  priv->rmt_dev->logic_one  = ((T1L << 16) | (0x8000 | T1H));
  priv->rmt_dev->logic_zero = ((T0L << 16) | (0x8000 | T0H));

  dev_data->open    = ws2812_rmt_open;
  dev_data->write   = ws2812_rmt_write;
  dev_data->close   = NULL;
  dev_data->read    = NULL;
  dev_data->nleds   = nleds;
  dev_data->clock   = CONFIG_WS2812_FREQUENCY;
  dev_data->private = priv;

  nxsem_init(&dev_data->exclsem, 0, 1);

  /* Register the WS2812 driver at the specified location. */

  snprintf(devpath, sizeof(devpath), "/dev/leds%d", devno);
  ret = ws2812_register(devpath, dev_data);
  if (ret < 0)
    {
      lederr("ERROR: ws2812_leds_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}
#  endif

#endif
