/**************************************************************************/
/*!
    @file     dcd_valentyusb.c
    @author   mithro

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Scott Shawcroft for Adafruit Industries
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
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include <irq.h>
#include <stdio.h>
#include <string.h>
#include <console.h>
#include <uart.h>

#include "generated/csr.h"

#include "tusb_option.h"

#include "device/dcd.h"

// #define printf(...) //_printf(__VA_ARGS__)
// const unsigned char _ctype[] = {
// 'x'
// };
// #include "printf.c"

#define ENDPOINT_BUF(num, dir) \
  (num << 1 | (epdir == TUSB_DIR_OUT ? 0 : 1))

#define ENDPOINT_NUM(epbuf) \
  (epbuf >> 1)

#define ENDPOINT_DIR(epbuf) \
  ((epbuf & 0x1) == 0 ? TUSB_DIR_OUT : TUSB_DIR_IN)

enum ENDPOINT_RESPONSE {
  ENDPOINT_ACK = 0b00,
  ENDPOINT_NAK = 0b01,
  ENDPOINT_STA = 0b11,
  ENDPOINT_IGN = 0b10,
};

enum TOK {
                //XX01
  TOK_OUT     = 0b00,
  TOK_SOF     = 0b01,
  TOK_IN      = 0b10,
  TOK_SETUP   = 0b11,
};

void printf_tok(enum TOK tok) {
  /*
    printf("tok ");
    switch(tok) {
    case TOK_OUT:
      printf("out ");
      break;
    case TOK_IN:
      printf("in ");
      break;
    case TOK_SOF:
      printf("sof ");
      break;
    case TOK_SETUP:
      printf("set ");
      break;
    default:
      printf("%x ", tok);
    }
    */
}

static uint8_t _setup_packet[8];

typedef struct {
  // Control functions
  void    (*set_response)(uint8_t);
  uint8_t (*get_response)(void);
  uint8_t (*pending_read)(void);
  void    (*pending_clear)(uint8_t);
  uint8_t (*last_token)(void);
  // FIFO
  union {
    struct {
      // Input FIFO
      uint8_t (*d2h_empty)(void);
      void    (*d2h_push)(uint8_t);
    } in;
    struct {
      // Output FIFO
      uint8_t (*h2d_empty)(void);
      uint8_t (*h2d_read)(void);
      void    (*h2d_pop)(uint8_t);
    } out;
  };
} ep_func_t;

ep_func_t ep_funcs[] = {

  // Endpoint 0 -- Out
  {
    // Control functions
    .set_response  = &usb_ep_0_out_respond_write,
    .get_response  = &usb_ep_0_out_respond_read,
    .pending_read  = &usb_ep_0_out_ev_pending_read,
    .pending_clear = &usb_ep_0_out_ev_pending_write,
    .last_token    = &usb_ep_0_out_last_tok_read,

    .out = {
      // Output FIFO
      .h2d_empty     = &usb_ep_0_out_obuf_empty_read,
      .h2d_read      = &usb_ep_0_out_obuf_head_read,
      .h2d_pop       = &usb_ep_0_out_obuf_head_write,
    },
  },

  // Endpoint 0 -- In
  {
    // Control functions
    .set_response  = &usb_ep_0_in_respond_write,
    .get_response  = &usb_ep_0_in_respond_read,
    .pending_read  = &usb_ep_0_in_ev_pending_read,
    .pending_clear = &usb_ep_0_in_ev_pending_write,
    .last_token    = &usb_ep_0_in_last_tok_read,

    // Input FIFO
    .in = {
      .d2h_empty     = &usb_ep_0_in_ibuf_empty_read,
      .d2h_push      = &usb_ep_0_in_ibuf_head_write,
    },
  },
#if 0
  // Endpoint 1 -- Out
#ifdef CSR_USB_EP_1_OUT_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_1_out_respond_write,
    .get_response  = &usb_ep_1_out_respond_read,
    .pending_read  = &usb_ep_1_out_ev_pending_read,
    .pending_clear = &usb_ep_1_out_ev_pending_write,
    .last_token    = &usb_ep_1_out_last_tok_read,

    .out = {
      // Output FIFO
      .h2d_empty     = &usb_ep_1_out_obuf_empty_read,
      .h2d_read      = &usb_ep_1_out_obuf_head_read,
      .h2d_pop       = &usb_ep_1_out_obuf_head_write,
    },
  },
#else
  {
    // Control functions
    .set_response  = NULL,
    .get_response  = NULL,
    .pending_read  = NULL,
    .pending_clear = NULL,
    .last_token    = NULL,

    .out = {
      // Output FIFO
      .h2d_empty     = NULL,
      .h2d_read      = NULL,
      .h2d_pop       = NULL,
    },
  },
#endif

  // Endpoint 1 -- In
#ifdef CSR_USB_EP_1_IN_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_1_in_respond_write,
    .get_response  = &usb_ep_1_in_respond_read,
    .pending_read  = &usb_ep_1_in_ev_pending_read,
    .pending_clear = &usb_ep_1_in_ev_pending_write,
    .last_token    = &usb_ep_1_in_last_tok_read,

    // Input FIFO
    .in = {
      .d2h_empty     = &usb_ep_1_in_ibuf_empty_read,
      .d2h_push      = &usb_ep_1_in_ibuf_head_write,
    },
  },
#else
  {
    // Control functions
    .set_response  = NULL,
    .get_response  = NULL,
    .pending_read  = NULL,
    .pending_clear = NULL,
    .last_token    = NULL,

    .out = {
      // Input FIFO
      .d2h_empty     = NULL,
      .d2h_push      = NULL,
    },
  },
#endif

  // Endpoint 2 -- Out
#ifdef CSR_USB_EP_2_OUT_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_2_out_respond_write,
    .get_response  = &usb_ep_2_out_respond_read,
    .pending_read  = &usb_ep_2_out_ev_pending_read,
    .pending_clear = &usb_ep_2_out_ev_pending_write,
    .last_token    = &usb_ep_2_out_last_tok_read,

    .out = {
      // Output FIFO
      .h2d_empty     = &usb_ep_2_out_obuf_empty_read,
      .h2d_read      = &usb_ep_2_out_obuf_head_read,
      .h2d_pop       = &usb_ep_2_out_obuf_head_write,
    },
  },
#else
  {
    // Control functions
    .set_response  = NULL,
    .get_response  = NULL,
    .pending_read  = NULL,
    .pending_clear = NULL,
    .last_token    = NULL,

    .out = {
      // Output FIFO
      .h2d_empty     = NULL,
      .h2d_read      = NULL,
      .h2d_pop       = NULL,
    },
  },
#endif

  // Endpoint 2 -- In
#ifdef CSR_USB_EP_2_IN_EV_STATUS_ADDR
  {
    // Control functions
    .set_response  = &usb_ep_2_in_respond_write,
    .get_response  = &usb_ep_2_in_respond_read,
    .pending_read  = &usb_ep_2_in_ev_pending_read,
    .pending_clear = &usb_ep_2_in_ev_pending_write,
    .last_token    = &usb_ep_2_in_last_tok_read,

    // Input FIFO
    .in = {
      .d2h_empty     = &usb_ep_2_in_ibuf_empty_read,
      .d2h_push      = &usb_ep_2_in_ibuf_head_write,
    },
  },
#else
  {
    // Control functions
    .set_response  = NULL,
    .get_response  = NULL,
    .pending_read  = NULL,
    .pending_clear = NULL,
    .last_token    = NULL,

    .out = {
      // Input FIFO
      .d2h_empty     = NULL,
      .d2h_push      = NULL,
    },
  },
#endif
#endif
};

struct {
  uint8_t* buffer;
  uint8_t  len;
} ep_buffers[sizeof(ep_funcs)/sizeof(ep_func_t)];



void isr(void);
__attribute__ ((used)) void isr(void)
{
	unsigned int irqs;

	irqs = irq_pending() & irq_getmask();

	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
}

void board_init(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();

  puts("Continue? ");
  readchar();
}

uint32_t tusb_hal_millis(void)
{
  return 0;
}


// Setup the control endpoint 0.
/*
static void bus_reset(void) {
    // Prepare for setup packet
    //dcd_edpt_xfer(0, 0, _setup_packet, sizeof(_setup_packet));
}*/

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/
bool dcd_init (uint8_t rhport)
{
  (void) rhport;

  // Clear all the buffer pointers.
  for (unsigned ep = 0; ep < (sizeof(ep_buffers)/sizeof(ep_buffers[0])); ep++) {
    ep_buffers[ep].buffer = NULL;
    ep_buffers[ep].len = 255;
  }

  // Allow the USB to start
  usb_pullup_out_write(0);
  usb_pullup_out_write(1);

  // Prepare for setup packet
  for (unsigned ep = 0; ep < (sizeof(ep_funcs)/sizeof(ep_funcs[0])); ep++) {
    if(ep_funcs[ep].pending_clear == NULL) {
      continue;
    }
    ep_funcs[ep].set_response(ENDPOINT_NAK);
    ep_funcs[ep].pending_clear(0xff);
  }
  printf("init\n");

  return true;
}

void dcd_connect (uint8_t rhport)
{
}
void dcd_disconnect (uint8_t rhport)
{
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;

  // Wait for EP0 to finish before switching the address.
}

void dcd_set_config (uint8_t rhport, uint8_t config_num)
{
  (void) rhport;
  (void) config_num;
  // Nothing to do
  printf("set_config #:%u\r\n", config_num);
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;
  (void) desc_edpt;

  const uint8_t epnum = edpt_number(desc_edpt->bEndpointAddress);
  const uint8_t epdir = edpt_dir(desc_edpt->bEndpointAddress);
  const uint8_t epbuf = ENDPOINT_BUF(epnum, epdir);

  printf("ep_%u_%s edpt_open\r\n", epnum, (epdir == TUSB_DIR_OUT) ? "out" : "in ");
  if (ep_funcs[epbuf].pending_clear == NULL) {
    printf("--ERR Invalid endpoint!");
    return false;
  }

  ep_funcs[epbuf].set_response(ENDPOINT_NAK);
  ep_buffers[epbuf].buffer = NULL;
  ep_buffers[epbuf].len = 255;
  return true;
}

int16_t dcd_out_transfer(const uint8_t epbuf, uint8_t* buffer, const uint16_t len)
{
  const uint8_t epnum = ENDPOINT_NUM(epbuf);
  const ep_func_t epf = ep_funcs[epbuf];
  (void)epnum;

  printf("ep_%u_out ", epnum);
  printf("w:%u ", len);

  if (len == 255) {
    printf(" -- ERR unexpected");
    return -1;
  }

  // Copy the data out of the FIFO
  uint16_t t = 0;
  while (!(epf.out.h2d_empty())) {
    if (t >= len) {
      break;
    }
    uint8_t byte = epf.out.h2d_read(); // Read the data
    epf.out.h2d_pop(0);                // Push the FIFO forward by one
    printf("%x ", byte);
    if (t < len) {
      buffer[t] = byte;
    }
    t++;
  }

  printf("g:%u ", t);
/*
  if (t < len) {
    printf("-- ERR short!");
  }
  if (t > len) {
    printf("-- ERR long!");
  }
  */
  return t;
}

void dcd_out_poll(const uint8_t epbuf)
{
  const uint8_t epnum = ENDPOINT_NUM(epbuf);
  const ep_func_t epf = ep_funcs[epbuf];
  if (!epf.pending_read()) {
    return;
  }

  while (!epf.out.h2d_empty()) {
    uint8_t* buffer = ep_buffers[epbuf].buffer;
    uint8_t  len    = ep_buffers[epbuf].len;
    // Clear the pointers
    ep_buffers[epbuf].buffer = NULL;
    ep_buffers[epbuf].len = 255;

    uint8_t last_tok = epf.last_token();
    if (len == 255) {
      if (last_tok == TOK_SETUP) {
        buffer = &(_setup_packet[0]);
        len = sizeof(_setup_packet);
        printf("-");
      }
    }
    printf_tok(last_tok);

    int16_t t = dcd_out_transfer(epbuf, buffer, len);
    if (t == -1) {
      printf(":-(\r\n");
      continue;
    }
    printf("\r\n");

    if (buffer != &(_setup_packet[0])) {
      dcd_event_xfer_complete(0, epnum, t, XFER_RESULT_SUCCESS, false);
    } else {
      dcd_event_setup_received(0, buffer, false);
    }
  }

  epf.set_response(ENDPOINT_NAK);
  epf.pending_clear(0xff);
}

void dcd_in_poll(uint8_t const epbuf)
{
  const uint8_t epnum = ENDPOINT_NUM(epbuf);
  const ep_func_t epf = ep_funcs[epbuf];
  if (!epf.pending_read()) {
    return;
  }

  if (!epf.in.d2h_empty()) {
    printf("in not empty\r\n!");
    return;
  }

  uint8_t  len = ep_buffers[epbuf].len;
  // Clear the pointers
  ep_buffers[epbuf].buffer = NULL;
  ep_buffers[epbuf].len = 255;

  printf_tok(epf.last_token());
  printf("ep_%u_in  ", epnum);
  printf("w:%u ", len);

  if (len == 255) {
    printf(" -- ERR unexpected");
    goto fail;
  }

  epf.set_response(ENDPOINT_NAK);
  epf.pending_clear(0xff);
  printf("!\r\n");

  dcd_event_xfer_complete(0, epnum | TUSB_DIR_IN_MASK, len, XFER_RESULT_SUCCESS, false);
  return;

fail:
  epf.set_response(ENDPOINT_STA);
  epf.pending_clear(0xff);
  printf(":-(\r\n");
}

void dcd_poll(uint8_t rhport)
{
  for (unsigned epbuf = 0; epbuf < (sizeof(ep_funcs)/sizeof(ep_funcs[0])); epbuf++) {
    const uint8_t epdir = ENDPOINT_DIR(epbuf);

    const ep_func_t epf = ep_funcs[epbuf];
    if (epf.pending_read == NULL) {
      continue;
    }

    if (epdir == TUSB_DIR_OUT) {
      dcd_out_poll(epbuf);
    } else {
      dcd_in_poll(epbuf);
    }
  }

}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  const uint8_t epnum = edpt_number(ep_addr);
  const uint8_t epdir = edpt_dir(ep_addr);
  const uint8_t epbuf = ENDPOINT_BUF(epnum, epdir);
  const ep_func_t epf = ep_funcs[epbuf];

  printf("[ ep:%u d:%s start xfer l:%u b:%p ", epnum, (epdir == TUSB_DIR_OUT) ? "o" : "i", total_bytes, buffer);

  // Endpoint is in use?
  if (epf.pending_read()) {
    printf("pp!\r\n");
    return false;
  }
  if (ep_buffers[epbuf].buffer != NULL) {
    // printf("ep.buffer != NULL (%p)\r\n", ep_buffers[epbuf].buffer);
    return false;
  }
  if (ep_buffers[epbuf].len != 255) {
    // printf("ep.len != 255 (%u)\r\n", ep_buffers[epbuf].len);
    return false;
  }
  if (epf.get_response() != ENDPOINT_NAK) {
    // printf("ep.state != NAK (%x)\r\n", epf.get_response());
    return false;
  }

  // Out transfer
  if ( epdir == TUSB_DIR_OUT ) {
    if (!epf.out.h2d_empty()) {
      printf("!empty\r\n");
      return false;
    }
    ep_buffers[epbuf].buffer = buffer;
    ep_buffers[epbuf].len = total_bytes;

  // In tranfer
  } else if ( epdir == TUSB_DIR_IN ) {
    if (!epf.in.d2h_empty()) {
      printf("!empty\r\n");
      return false;
    }
    // FIXME: Check total_bytes...

    // Push the data into the outgoing FIFO
    for(uint16_t i = 0; i < total_bytes; i++) {
      printf("%x ", buffer[i]);
      epf.in.d2h_push(buffer[i]);
    }
    if (total_bytes > 0) {
      if(epf.in.d2h_empty()) {
        printf("-ERR ep empty!?\r\n");
        return false;
      }
    }
    if (epf.pending_read()) {
      printf("-ERR ep pending!?\r\n");
      return false;
    }

    ep_buffers[epbuf].buffer = NULL;
    ep_buffers[epbuf].len = total_bytes;

  } else {
    printf("ERR - Unknown EP direction\r\n");
    return false;
  }

  epf.set_response(ENDPOINT_ACK);
  printf(" ..\r\n");
  return true;
}

bool dcd_edpt_stalled (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  const uint8_t epnum = edpt_number(ep_addr);
  const uint8_t epdir = edpt_dir(ep_addr);
  const uint8_t epbuf = ENDPOINT_BUF(epnum, epdir);
  return (ep_funcs[epbuf].get_response() == ENDPOINT_STA);
}

void dcd_edpt_set_response(uint8_t rhport, uint8_t ep_addr, enum ENDPOINT_RESPONSE e)
{
  (void) rhport;
  const uint8_t epnum = edpt_number(ep_addr);
  const uint8_t epdir = edpt_dir(ep_addr);
  const uint8_t epbuf = ENDPOINT_BUF(epnum, epdir);
  ep_funcs[epbuf].set_response(e);
  printf("stall ep:%u d:%s %x\r\n", epnum, (epdir == TUSB_DIR_OUT) ? "o" : "i", e);
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  dcd_edpt_set_response(rhport, ep_addr, ENDPOINT_STA);
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  printf("un");
  dcd_edpt_set_response(rhport, ep_addr, ENDPOINT_NAK);
}

bool dcd_edpt_busy (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  //const uint8_t epnum = edpt_number(ep_addr);
  //const uint8_t epdir = edpt_dir(ep_addr);
  //const uint8_t epbuf = ENDPOINT_BUF(epnum, epdir);
  //return (ep_buffers[epbuf].len != 255) || (ep_funcs[epbuf].pending_read());
  return false;
}

/*------------------------------------------------------------------*/
