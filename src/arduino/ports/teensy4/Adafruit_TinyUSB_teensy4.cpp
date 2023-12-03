/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach for Adafruit
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
 */

#include "tusb_option.h"

#if (defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)) && CFG_TUD_ENABLED

#include "Arduino.h"
#include "arduino/Adafruit_TinyUSB_API.h"
#include "tusb.h"
//#include <Reset.h> // Needed for auto-reset with 1200bps port touch

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
extern "C"
{
void USB_Handler(void) { tud_int_handler(0); }
} // extern C

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
static void usb_hardware_init(void);

void TinyUSB_Port_InitDevice(uint8_t rhport)
{
  (void)rhport;

  // USBSerial.setStringDescriptor("TinyUSB Serial");
  // USBDevice.addInterface(USBSerial);
  // USBDevice.setID(USB_VID, USB_PID);
  // USBDevice.begin();

  usb_hardware_init();

  // Init tinyusb stack
  tud_init(0);
}

uint8_t TinyUSB_Port_GetSerialNumber(uint8_t serial_id[16])
{
  char buf[11];
  uint32_t i, num;

  num = HW_OCOTP_MAC0 & 0xFFFFFF;
  // add extra zero to work around OS-X CDC-ACM driver bug
  if (num < 10000000) num = num * 10;
  ultoa(num, buf, 10);

  for (i=0; i<10; i++) {
    char c = buf[i];
    if (!c) break;
    serial_id[i] = c;
  }

  return i * 2 + 2;
}

// Init usb hardware when starting up. Softdevice is not enabled yet
static void usb_hardware_init(void)
{
  PMU_REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) | PMU_REG_3P0_BO_OFFSET(6)
    | PMU_REG_3P0_ENABLE_LINREG;
  
  CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON); // turn on clocks to USB peripheral
#if 1
  if ((USBPHY1_PWD & (USBPHY_PWD_RXPWDRX | USBPHY_PWD_RXPWDDIFF | USBPHY_PWD_RXPWD1PT1
    | USBPHY_PWD_RXPWDENV | USBPHY_PWD_TXPWDV2I | USBPHY_PWD_TXPWDIBIAS
    | USBPHY_PWD_TXPWDFS)) || (USB1_USBMODE & USB_USBMODE_CM_MASK)) {
    // USB controller is turned on from previous use
    // reset needed to turn it off & start from clean slate
    USBPHY1_CTRL_SET = USBPHY_CTRL_SFTRST; // USBPHY1_CTRL page 3292
    NVIC_CLEAR_PENDING(IRQ_USB1);
    USBPHY1_CTRL_CLR = USBPHY_CTRL_SFTRST; // reset PHY
    //USB1_USBSTS = USB1_USBSTS; // TODO: is this needed?
    delay(25);
  }
#endif

  USBPHY1_CTRL_CLR = USBPHY_CTRL_CLKGATE;
  USBPHY1_PWD = 0;
  
  attachInterruptVector(IRQ_USB1, &USB_Handler);
  NVIC_SET_PRIORITY(IRQ_USB1, 0);

#endif