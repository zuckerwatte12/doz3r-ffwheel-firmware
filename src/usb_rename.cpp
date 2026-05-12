#include "usb_rename.h"

static bool SendControl(u8 d) {
    return USB_SendControl(0, &d, 1) == 1;
}

static bool USB_SendStringDescriptor(const char* string_P, size_t string_len, uint8_t flags) {
    SendControl(2 + string_len * 2);
    SendControl(3);
    bool pgm = flags & TRANSFER_PGM;

    for (size_t i = 0; i < string_len; i++) {
        bool r = SendControl(pgm ? pgm_read_byte(&string_P[i]) : (u8)string_P[i]);
        r &= SendControl(0);
        if (!r) return false;
    }
    return true;
}

USBRename::USBRename(
    const char *product_name,
    const char *manufacturer_name,
    const char *serial_num,
    uint16_t vid,
    uint16_t pid
) :
    product_name(product_name),
    manufacturer_name(manufacturer_name),
    serial_num(serial_num),
    vid(vid),
    pid(pid),
    PluggableUSBModule(1, 1, epType)
{
    PluggableUSB().plug(this);
}

int USBRename::getInterface(uint8_t* interfaceCount) {
    return 0;
}

int USBRename::getDescriptor(USBSetup& setup)
{
    // Handle string descriptors
    if (setup.wValueH == USB_STRING_DESCRIPTOR_TYPE) {
        if (setup.wValueL == IMANUFACTURER && manufacturer_name)
            return USB_SendStringDescriptor(manufacturer_name, strlen(manufacturer_name), 0);

        if (setup.wValueL == IPRODUCT && product_name)
            return USB_SendStringDescriptor(product_name, strlen(product_name), 0);

        if (setup.wValueL == ISERIAL && serial_num)
            return USB_SendStringDescriptor(serial_num, strlen(serial_num), 0);

        return 0;
    }

    // Handle device descriptor (VID/PID override)
    if (setup.wValueH == USB_DEVICE_DESCRIPTOR_TYPE && setup.wValueL == 0) {

        uint8_t devDesc[] = {
            18,                       // bLength
            USB_DEVICE_DESCRIPTOR_TYPE,
            0x00, 0x02,               // USB version 2.00
            0x00,                     // class
            0x00,                     // subclass
            0x00,                     // protocol
            USB_EP_SIZE,              // max packet size
            (uint8_t)(vid & 0xFF),    // VID low
            (uint8_t)(vid >> 8),      // VID high
            (uint8_t)(pid & 0xFF),    // PID low
            (uint8_t)(pid >> 8),      // PID high
            0x00, 0x01,               // device version
            IMANUFACTURER,
            IPRODUCT,
            ISERIAL,
            1                          // number of configurations
        };

        return USB_SendControl(0, devDesc, sizeof(devDesc));
    }

    return 0;
}
bool USBRename::setup(USBSetup& setup)
{
    return false;
}