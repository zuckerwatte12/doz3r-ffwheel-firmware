#include "PluggableUSB.h"

class USBRename : public PluggableUSBModule {
  public:
    USBRename(
      const char *product_name = NULL,
      const char *manufacturer_name = NULL,
      const char *serial_num = NULL,
      uint16_t vid = 0x2341,   // default Arduino VID
      uint16_t pid = 0x0001    // default PID
    );

    int getInterface(uint8_t* interfaceCount);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);

  private:
    const char *manufacturer_name;
    const char *product_name;
    const char *serial_num;

    uint16_t vid;
    uint16_t pid;

    uint8_t epType[0];
};
