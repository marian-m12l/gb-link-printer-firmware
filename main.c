/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include <stdio.h>
#include <string.h>
#include "tusb.h"
#include "hardware/pio.h"


#ifdef SECONDARY_MODE
  #include "pio/pio_secondary.h"
#else
  #include "pio/pio_spi.h"
#endif
const uint SI_PIN = 3;
//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
#ifdef SECONDARY_MODE
  BLINK_MOUNTED     = 500,
#else
  BLINK_MOUNTED     = 1000,
#endif
  BLINK_SUSPENDED   = 2500,

  BLINK_ALWAYS_ON   = UINT32_MAX,
  BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

#ifdef SECONDARY_MODE
#define URL  "localhost:3001"
#else
#define URL  "localhost:3000"
#endif

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;

//------------- prototypes -------------//
void led_blinking_task(void);
void cdc_task(void);
void webserial_task(void);
#ifdef SECONDARY_MODE
void secondary_task(void);
#endif

/*------------- MAIN -------------*/

#ifdef SECONDARY_MODE
  pio_secondary_inst_t secondary = {
          .pio = pio1,
          .sm = 0
  };
#else
  pio_spi_inst_t spi = {
          .pio = pio1,
          .sm = 0
  };
#endif



#define PIN_SCK 0
#define PIN_SIN 1
#define PIN_SOUT 2

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main(void)
{
  #ifdef SECONDARY_MODE
    uint secondary_prog_offs = pio_add_program(secondary.pio, &secondary_program);
    secondary_program_init(secondary.pio, secondary.sm, secondary_prog_offs, PIN_SCK, PIN_SIN, PIN_SOUT);
  #else
    uint cpha1_prog_offs = pio_add_program(spi.pio, &spi_cpha1_program);
    pio_spi_init(spi.pio, spi.sm, cpha1_prog_offs, 8, 4058.838, 1, 1, PIN_SCK, PIN_SOUT, PIN_SIN);
  #endif



  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
    led_blinking_task();
    #ifdef SECONDARY_MODE
      secondary_task();                                                                                                                           
    #endif
  }

  return 0;
}

int oldmain(void)
{
  board_init();

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
    led_blinking_task();
  }

  return 0;
}

// send characters to both CDC and WebUSB
void echo_all(uint8_t buf[], uint32_t count)
{
  // echo to web serial
  if ( web_serial_connected )
  {
    tud_vendor_write(buf, count);
  }

  // echo to cdc
  if ( tud_cdc_connected() )
  {
    for(uint32_t i=0; i<count; i++)
    {
      tud_cdc_write_char(buf[i]);

      if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
    }
    tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when received VENDOR control request
bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  switch (request->bRequest)
  {
    case VENDOR_REQUEST_WEBUSB:
      // match vendor request in BOS descriptor
      // Get landing page url
      return tud_control_xfer(rhport, request, (void*) &desc_url, desc_url.bLength);

    case VENDOR_REQUEST_MICROSOFT:
      if ( request->wIndex == 7 )
      {
        // Get Microsoft OS 2.0 compatible descriptor
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20+8, 2);

        return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
      }else
      {
        return false;
      }

    case 0x22:
      // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to
      // connect and disconnect.
      web_serial_connected = (request->wValue != 0);

      // Always lit LED if connected
      if ( web_serial_connected )
      {
        //board_led_write(true);
        gpio_put(LED_PIN, 1);
        blink_interval_ms = BLINK_ALWAYS_ON;

        // tud_vendor_write_str("\r\nTinyUSB WebUSB device example\r\n");
      }else
      {
        blink_interval_ms = BLINK_MOUNTED;
      }

      // response with status OK
      return tud_control_status(rhport, request);

    default:
      // stall unknown request
      return false;
  }

  return true;
}

// Invoked when DATA Stage of VENDOR's request is complete
bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;

  // nothing to do
  return true;
}

void webserial_task(void)
{
  if ( web_serial_connected )
  {
    if ( tud_vendor_available() )
    {
      uint8_t buf[1];
      uint32_t count = tud_vendor_read(buf, sizeof(buf));
      if(count) {
        // pprintf("Sending: %02x", buf[0]);

        #ifdef SECONDARY_MODE
          pio_secondary_write8_blocking(&secondary, buf, count); // Stores in outgoing FIFO to be sent whenever the primary device issues clock ticks
        #else
          unsigned char rx;
          pio_spi_write8_read8_blocking(&spi, buf, &rx, 1);
          echo_all(&rx, 1);
        #endif

      }
      // echo back to both web serial and cdc
      // echo_all(buf, count);
    }
  }
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if ( tud_cdc_available() )
    {
      
      uint8_t buf[1];
      uint32_t count = tud_vendor_read(buf, sizeof(buf));
      if(count) {

        #ifdef SECONDARY_MODE
          pio_secondary_write8_blocking(&secondary, buf, count); // Stores in outgoing FIFO to be sent whenever the primary device issues clock ticks
        #else
          unsigned char rx;
          pio_spi_write8_read8_blocking(&spi, buf, &rx, 1);
          echo_all(&rx, 1);
        #endif
      }
      // echo back to both web serial and cdc
      // echo_all(buf, count);
    
      // uint8_t buf[64];

      // uint32_t count = tud_cdc_read(buf, sizeof(buf));

      // // echo back to both web serial and cdc
      // echo_all(buf, count);
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;

  // connected
  if ( dtr && rts )
  {
    // print initial message when connected
    // tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  //board_led_write(led_state);
  gpio_put(LED_PIN, led_state);
  led_state = 1 - led_state; // toggle
}


#ifdef SECONDARY_MODE
  void secondary_task(void)
  {
    // Is data available on PIO ?
    if (pio_secondary_available(&secondary)) {
      // Read byte
      //uint8_t rx = pio_secondary_read8(&secondary); // TODO Use buffer as argument instead of return value ???
      uint8_t rx;
      pio_secondary_read8(&secondary, &rx, 1);
      //pio_secondary_read8_blocking(&secondary, &rx, 1);
  
      // Send byte to both web serial and cdc
      echo_all(&rx, 1);
    }
  }
#endif
