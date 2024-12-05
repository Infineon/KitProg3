### KitProg3 v2.70.0 Release Notes

KitProg3 is our low-level communication firmware for programming and debugging. It provides communication between a programming tool (such as CYPRESS™ Programmer or PSoC™ Programmer) and a target, such as a PSoC™ 6 MCU. KitProg3 supports a variety of development kits. It is also the communication firmware found in the MiniProg4 debug probe.
Our development kits have KitProg3 firmware installed to provide the necessary communication between the host and target. As a result, when you plug the kit into your host computer, programming and debugging just work.

### New Features

- Added support of DAPLink mode on CY8CKIT-062S2-AI PSOC™ 6 AI Evaluation Kit

### Known Issues

| ID                                | Known Issue                       | Workaround                          |
|-----------------------------------|-----------------------------------|-----------------------------------|
|  PROGTOOLS-768 | KitProg3 firmware might not be updated properly on a particular USB port on some Lenovo ThinkPad Pro Docking Station.  |  If you encounter this problem: 1. switch to a different USB port on the docking station OR 2. Update KitProg3 firmware in Bootloader mode (press the mode switch while plugging in the board). |
|  PROGTOOLS-1488 | Junk characters might be observed in a UART terminal during programming of the connected kit or after programming is competed. | Clear UART buffers after kit programming is completed.   |
|  PROGTOOLS-1838 | Starting from KitProg3 v2.10, when KitProg3 is in CMSIS-DAP Bulk mode, it is not possible to debug and use USB-I2C/SPI bridging (for example, in the CapSense Tuner, Bridge Control Panel) at the same time. This affects Windows OS only. It does not affect Linux or macOS users. | If you would like to use debug and USB-I2C/SPI bridging at the same time, there are two possible workarounds: <br />1. If performance for programming and debug is not critical,switch KitProg3 to CMSIS-DAP. <br />2. If you need faster performance for programming and debug, use the onboard KitProg3 for programming purposes and MiniProg4 for bridging purposes or vice versa. Both devices can be in CMSIS-DAP bulk mode. Details are in [KBA231025](https://community.infineon.com/t5/Knowledge-Base-Articles/Windows-Only-With-KitProg3-v2-10-Simultaneous-Use-of-USB-I2C-SPI/ta-p/250449). |
|  PROGTOOLS-1869 | In Linux OS, with KitProg3 in CMSIS-DAP HID mode, a debug session in ModusToolbox™ can be destroyed if you use the Firmware Loader --device-list command while debugging. This is limitation of hidapi library used on Linux. macOS and Windows OSes are not impacted. | If you have a debug session running, don't use the firmware loader tool.  |
|  PROGTOOLS-1814 | Starting from KitProg3 v2.10, in some cases Windows 7 does not recognize the KitProg3 bridge. So the USB- I2C/SPI bridge devices are not available in either CMSIS-DAP HID or CMSIS-DAP bulk mode. | Install a digitally signed driver manually from the Windows Update Catalog. Follow steps from [KBA231026](https://community.infineon.com/t5/Knowledge-Base-Articles/Windows-7-No-USB-I2C-SPI-Bridge-Device-Available-when-KitProg3/ta-p/250443). |



### More information

-   [fw-loader
    README.md](https://github.com/Infineon/Firmware-loader/blob/master/README.md)

-   [Fw-loader Release Notes](https://github.com/Infineon/Firmware-loader/blob/master/RELEASE.MD)

-   [Kitprog3 User
    Guide](https://www.infineon.com/documentation/development-kitsboards/kitprog-user-guide)

-   [KitProg Host Protocol Interface](https://www.infineon.com/dgdl/Infineon-KitProg_Host_Protocol_Interface-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f0125c8185e)

-   [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and
    Videos](https://www.infineon.com/modustoolbox)

-   [Cypress Semiconductor, an Infineon Technologies Company](http://www.infineon.com)

© Cypress Semiconductor Corporation, 2019-2024. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").
