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

#include "board_api.h"
#include "tusb.h"

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define PID_MAP(itf, n)  ((CFG_TUD_##itf) ? (1 << (n)) : 0)
#define USB_PID           (0x4000 | PID_MAP(CDC, 0) | PID_MAP(MSC, 1) | PID_MAP(HID, 2) | \
                           PID_MAP(MIDI, 3) | PID_MAP(VENDOR, 4) )

#define USB_VID   0xCafe
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = 0x4005,//USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x00, //0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_CDC_0 = 0,
  ITF_NUM_CDC_0_DATA,
//  ITF_NUM_CDC_1,
//  ITF_NUM_CDC_1_DATA,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X || CFG_TUSB_MCU == OPT_MCU_LPC40XX
  // LPC 17xx and 40xx endpoint type (bulk/interrupt/iso) are fixed by its number
  // 0 control, 1 In, 2 Bulk, 3 Iso, 4 In etc ...
  #define EPNUM_CDC_0_NOTIF   0x81
  #define EPNUM_CDC_0_OUT     0x02
  #define EPNUM_CDC_0_IN      0x82

  #define EPNUM_CDC_1_NOTIF   0x84
  #define EPNUM_CDC_1_OUT     0x05
  #define EPNUM_CDC_1_IN      0x85

#elif CFG_TUSB_MCU == OPT_MCU_CXD56
  // CXD56 USB driver has fixed endpoint type (bulk/interrupt/iso) and direction (IN/OUT) by its number
  // 0 control (IN/OUT), 1 Bulk (IN), 2 Bulk (OUT), 3 In (IN), 4 Bulk (IN), 5 Bulk (OUT), 6 In (IN)
  #define EPNUM_CDC_0_NOTIF   0x83
  #define EPNUM_CDC_0_OUT     0x02
  #define EPNUM_CDC_0_IN      0x81

  #define EPNUM_CDC_1_NOTIF   0x86
  #define EPNUM_CDC_1_OUT     0x05
  #define EPNUM_CDC_1_IN      0x84

#elif defined(TUD_ENDPOINT_ONE_DIRECTION_ONLY)
  // MCUs that don't support a same endpoint number with different direction IN and OUT defined in tusb_mcu.h
  //    e.g EP1 OUT & EP1 IN cannot exist together
  #define EPNUM_CDC_0_NOTIF   0x81
  #define EPNUM_CDC_0_OUT     0x02
  #define EPNUM_CDC_0_IN      0x83

  #define EPNUM_CDC_1_NOTIF   0x84
  #define EPNUM_CDC_1_OUT     0x05
  #define EPNUM_CDC_1_IN      0x86

#else
  #define EPNUM_CDC_0_NOTIF   0x81
  #define EPNUM_CDC_0_OUT     0x02
  #define EPNUM_CDC_0_IN      0x82

//  #define EPNUM_CDC_1_NOTIF   0x83
//  #define EPNUM_CDC_1_OUT     0x04
//  #define EPNUM_CDC_1_IN      0x84
#endif


static uint8_t const desc_fs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  //TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 3, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
//  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 16, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
// Per USB specs: high speed capable device must report device_qualifier and other_speed_configuration
static uint8_t const desc_hs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 512),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 16, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 512),
};

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
static tusb_desc_device_qualifier_t const desc_device_qualifier = {
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
// device_qualifier descriptor describes information about a high-speed capable device that would
// change if the device were operating at the other speed. If not highspeed capable stall this request.
uint8_t const *tud_descriptor_device_qualifier_cb(void) {
  return (uint8_t const *) &desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Configuration descriptor in the other speed e.g if high speed then this is for full speed and vice versa
uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index) {
  (void) index;// for multiple configurations

  // if link speed is high return fullspeed config, and vice versa
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration : desc_hs_configuration;
}

#endif// highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

// array of pointer to string descriptors
static char const *string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "TinyUSB",                     // 1: Manufacturer
  "TinyUSB Device",              // 2: Product
  //NULL,                          // 3: Serials will use unique ID if possible
  "LoRa Test Platform",                 // 4: CDC Interface
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch ( index ) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) { return NULL; }

      const char *str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if ( chr_count > max_count ) { chr_count = max_count; }

      // Convert ASCII string into UTF-16
      for ( size_t i = 0; i < chr_count; i++ ) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}


//#define BOS_TOTAL_LEN 17
//#define MS_OS_20_DESC_LEN  0xB2 // Exact length of our custom OS 2.0 descriptor block
//
//// 1. Update your BOS descriptor to advertise MS OS 2.0 support
//uint8_t const desc_bos[] = {
//  // Total length, number of device caps
//  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
//  // Vendor Code = 0x01, Total Length of MS OS 2.0 descriptor
//  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 0x01)
//};
//
//uint8_t const *tud_descriptor_bos_cb(void) {
//  return desc_bos;
//}
//
//// 2. Create the MS OS 2.0 Descriptor Array
//uint8_t const desc_ms_os_20[] = {
//  // Set header: length, type, windows version (Windows 8.1+), total length
//  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
//
//  // Configuration subset header
//  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),
//
//  // Function subset header (Binds to your CDC interface, e.g., Interface 0)
//  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_CDC_0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),
//
//  // Registry property feature: This forces the Custom Name into Device Manager!
//  // Length (0x54), Type (0x04), Property Data Type (REG_SZ = 1), Name Length (0x28)
//  U16_TO_U8S_LE(0x0054), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(1), U16_TO_U8S_LE(0x0028),
//  // Property Name in UTF-16LE: "DeviceInterfaceGUIDs" (Required for virtual registry injection)
//  'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0, 'I',0, 'n',0, 't',0, 'e',0, 'r',0, 'f',0, 'a',0, 'c',0, 'e',0, 'G',0, 'U',0, 'I',0, 'D',0, 's',0, 0,0,
//  // Property Data Length (0x26)
//  U16_TO_U8S_LE(0x0026),
//  // Dummy GUID value string in UTF-16LE: "{13032338-0000-0000-0000-000000000000}"
//  '{',0,'1',0,'3',0,'0',0,'3',0,'2',0,'3',0,'3',0,'8',0,'-',0,'0',0,'0',0,'0',0,'0',0,'-',0,'0',0,'0',0,'0',0,'0',0,'-',0,'0',0,'0',0,'0',0,'0',0,'-',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'}',0, 0,0,
//
//  // Second Registry Property: "PortName" or "DeviceDesc" override
//  // Length (0x3C), Type (0x04), Property Data Type (REG_SZ = 1), Name Length (0x16)
//  U16_TO_U8S_LE(0x0040), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), U16_TO_U8S_LE(1), U16_TO_U8S_LE(0x0016),
//  // Property Name: "Label" or "DeviceDesc" (DeviceDesc overrides Windows' default naming convention)
//  'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0, 'D',0, 'e',0, 's',0, 'c',0, 0,0,
//  U16_TO_U8S_LE(0x001A),  // Data Length (26 bytes)
//  // Custom name string in UTF-16LE: "TinyUSB Device"
//  'T',0, 'i',0, 'n',0, 'y',0, 'U',0, 'S',0, 'B',0, ' ',0, 'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 0,0
//};
//
//// Callback required by TinyUSB
//uint8_t const *tud_descriptor_ms_os_20_cb(void) {
//  return desc_ms_os_20;
//}

