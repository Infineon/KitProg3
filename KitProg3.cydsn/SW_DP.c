/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        1. December 2017
 * $Revision:    V2.0.0
 *
 * Project:      CMSIS-DAP Source
 * Title:        SW_DP.c CMSIS-DAP SW DP I/O
 *
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * Portions Copyright 2018-2021, Cypress Semiconductor Corporation
 * (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
 * All rights reserved.
 *
 * This software, associated documentation and materials (“Software”) is
 * owned by Cypress Semiconductor Corporation or one of its
 * affiliates (“Cypress”) and is protected by and subject to worldwide
 * patent protection (United States and foreign), United States copyright
 * laws and international treaty provisions. Therefore, you may use this
 * Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software (“EULA”). If
 * no EULA applies, then any reproduction, modification, translation,
 * compilation, or representation of this Software is prohibited without the
 * express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
 * BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE. Cypress reserves the right to make
 * changes to the Software without notice. Cypress does not assume any
 * liability arising out of the application or use of the Software or any
 * product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * THE CYPRESS COPYRIGHTED PORTIONS ARE NOT SUBMISSIONS AS SET FORTH IN THE
 * APACHE LICENSE VERSION 2.0 OR ANY OTHER LICENSE HAVING SIMILAR PROVISIONS.
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"


// SW Macros

#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)


// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
#if ((DAP_SWD != 0) || (DAP_JTAG != 0))
void SWJ_Sequence (uint32_t count, const uint8_t *data) {
  uint32_t val;
  uint32_t n;

  val = 0U;
  n = 0U;
  while (count--) {
    if (n == 0U) {
      val = *data++;
      n = 8U;
    }
    if (val & 1U) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
}
#endif


// Generate SWD Sequence
//   info:   sequence information
//   swdo:   pointer to SWDIO generated data
//   swdi:   pointer to SWDIO captured data
//   return: none
#if (DAP_SWD != 0)
void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
  uint32_t val;
  uint32_t bit;
  uint32_t n, k;

  n = info & SWD_SEQUENCE_CLK;
  if (n == 0U) {
    n = 64U;
  }

  if (info & SWD_SEQUENCE_DIN) {
    while (n) {
      val = 0U;
      for (k = 8U; k && n; k--, n--) {
        SW_READ_BIT(bit);
        val >>= 1;
        val  |= bit << 7;
      }
      val >>= k;
      *swdi++ = (uint8_t)val;
    }
  } else {
    while (n) {
      val = *swdo++;
      for (k = 8U; k && n; k--, n--) {
        SW_WRITE_BIT(val);
        val >>= 1;
      }
    }
  }
}
#endif


#if (DAP_SWD != 0)


// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
#define SWD_TransferFunction(speed)     /**/                                    \
static uint8_t SWD_Transfer##speed (uint32_t request, uint32_t *data) {         \
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t parity;                                                              \
                                                                                \
  uint32_t n;                                                                   \
                                                                                \
  /* Packet Request */                                                          \
  parity = 0U;                                                                  \
  SW_WRITE_BIT(1U);                     /* Start Bit */                         \
  bit = request >> 0;                                                           \
  SW_WRITE_BIT(bit);                    /* APnDP Bit */                         \
  parity += bit;                                                                \
  bit = request >> 1;                                                           \
  SW_WRITE_BIT(bit);                    /* RnW Bit */                           \
  parity += bit;                                                                \
  bit = request >> 2;                                                           \
  SW_WRITE_BIT(bit);                    /* A2 Bit */                            \
  parity += bit;                                                                \
  bit = request >> 3;                                                           \
  SW_WRITE_BIT(bit);                    /* A3 Bit */                            \
  parity += bit;                                                                \
  SW_WRITE_BIT(parity);                 /* Parity Bit */                        \
  SW_WRITE_BIT(0U);                     /* Stop Bit */                          \
  SW_WRITE_BIT(1U);                     /* Park Bit */                          \
                                                                                \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    SW_CLOCK_CYCLE();                                                           \
  }                                                                             \
                                                                                \
  /* Acknowledge response */                                                    \
  SW_READ_BIT(bit);                                                             \
  ack  = bit << 0;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 1;                                                              \
  SW_READ_BIT(bit);                                                             \
  ack |= bit << 2;                                                              \
                                                                                \
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       \
    /* Data transfer */                                                         \
    if (request & DAP_TRANSFER_RnW) {                                           \
      /* Read data */                                                           \
      val = 0U;                                                                 \
      parity = 0U;                                                              \
      for (n = 32U; n; n--) {                                                   \
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  \
        parity += bit;                                                          \
        val >>= 1;                                                              \
        val  |= bit << 31;                                                      \
      }                                                                         \
      SW_READ_BIT(bit);                 /* Read Parity */                       \
      if ((parity ^ bit) & 1U) {                                                \
        ack = DAP_TRANSFER_ERROR;                                               \
      }                                                                         \
      if (data) { *data = val; }                                                \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
    } else {                                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
      /* Write data */                                                          \
      val = *data;                                                              \
      parity = 0U;                                                              \
      for (n = 32U; n; n--) {                                                   \
        SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                 \
        parity += val;                                                          \
        val >>= 1;                                                              \
      }                                                                         \
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  \
    }                                                                           \
    /* Capture Timestamp */                                                     \
    if (request & DAP_TRANSFER_TIMESTAMP) {                                     \
      DAP_Data.timestamp = TIMESTAMP_GET();                                     \
    }                                                                           \
    /* Idle cycles */                                                           \
    n = DAP_Data.transfer.idle_cycles;                                          \
    if (n) {                                                                    \
      PIN_SWDIO_OUT(0U);                                                        \
      for (; n; n--) {                                                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1U);                                                          \
    return ((uint8_t)ack);                                                      \
  }                                                                             \
                                                                                \
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              \
    /* WAIT or FAULT response */                                                \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) { \
      for (n = 32U+1U; n; n--) {                                                \
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   \
      }                                                                         \
    }                                                                           \
    /* Turnaround */                                                            \
    for (n = DAP_Data.swd_conf.turnaround; n; n--) {                            \
      SW_CLOCK_CYCLE();                                                         \
    }                                                                           \
    PIN_SWDIO_OUT_ENABLE();                                                     \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) { \
      PIN_SWDIO_OUT(0U);                                                        \
      for (n = 32U+1U; n; n--) {                                                \
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1U);                                                          \
    return ((uint8_t)ack);                                                      \
  }                                                                             \
                                                                                \
  /* Protocol error */                                                          \
  for (n = DAP_Data.swd_conf.turnaround + 32U + 1U; n; n--) {                   \
    SW_CLOCK_CYCLE();                   /* Back off data phase */               \
  }                                                                             \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  PIN_SWDIO_OUT(1U);                                                            \
  return ((uint8_t)ack);                                                        \
}


#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_FAST()
SWD_TransferFunction(Fast)

#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)
SWD_TransferFunction(Slow)

// Pointer to actual SWD Transfer function
static uint8_t (*SWD_Transfer_ptr)(uint32_t request, uint32_t *data) = &SWD_TransferSlow;

// Switch among SWD Transfer implementation
void setSWD_Transfer_ptr(uint8_t mode)
{
    switch(mode)
    {
        case SWD_IMPL_HW:
            SWD_Transfer_ptr = &Swd_TransferHw;
            break;
        case SWD_IMPL_SW_FAST:
            SWD_Transfer_ptr = &SWD_TransferFast;
            break;
        default:
            SWD_Transfer_ptr = &SWD_TransferSlow;
            break;
    }
}

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data) {
    return (*SWD_Transfer_ptr)(request, data);
}


#endif  /* (DAP_SWD != 0) */
