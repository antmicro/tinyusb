/**************************************************************************/
/*!
    @file     main.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
void print_greeting(void);
void led_blinking_task(void);
void virtual_com_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  print_greeting();

  //tusb_init();

  while (1)
  {
    //tusb_task();

    led_blinking_task();
    //virtual_com_task();
  }

  return 0;
}

void virtual_com_task(void)
{
  // connected and there are data available
  if ( tud_mounted() && tud_cdc_available() )
  {
    uint8_t buf[64];

    // read and echo back
    uint32_t count = tud_cdc_read(buf, sizeof(buf));

    tud_cdc_write(buf, count);
  }
}


//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+
void tud_mount_cb(uint8_t port)
{

}

void tud_umount_cb(uint8_t port)
{
}

void tud_cdc_rx_cb(uint8_t port)
{
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  enum { BLINK_INTEVAL = 1000 };

  static uint32_t led_on_mask = 0;
  static uint32_t last_blink = 0;

  // not enough time
  if ( last_blink + BLINK_INTEVAL > tusb_hal_tick_get() ) return;

  last_blink += BLINK_INTEVAL;

  board_leds(led_on_mask, 1 - led_on_mask);
  led_on_mask = 1 - led_on_mask; // toggle
}

//--------------------------------------------------------------------+
// HELPER FUNCTION
//--------------------------------------------------------------------+
void print_greeting(void)
{
  char const * const rtos_name[] =
  {
      [TUSB_OS_NONE]      = "None",
      [TUSB_OS_FREERTOS]  = "FreeRTOS",
  };

  printf("\n--------------------------------------------------------------------\n");
  printf("-                     Device Demo (a tinyusb example)\n");
  printf("- if you find any bugs or get any questions, feel free to file an\n");
  printf("- issue at https://github.com/hathach/tinyusb\n");
  printf("--------------------------------------------------------------------\n\n");

  printf("This DEVICE demo is configured to support:");
  printf("  - RTOS = %s\n", rtos_name[TUSB_CFG_OS]);
  if (TUSB_CFG_DEVICE_HID_MOUSE    ) puts("  - HID Mouse");
  if (TUSB_CFG_DEVICE_HID_KEYBOARD ) puts("  - HID Keyboard");
  if (TUSB_CFG_DEVICE_MSC          ) puts("  - Mass Storage");
  if (TUSB_CFG_DEVICE_CDC          ) puts("  - Communication Device Class");
}