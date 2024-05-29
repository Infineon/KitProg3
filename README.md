# KitProg3
KitProg3 Firmware repo

### Overview

This repo contains only KitProg3 Firmware source code.
The compiled firmware is delivered with [fw-loader utility](https://github.com/Infineon/Firmware-loader).

### Build Notes

- This project is built with [PSoC Creator v.4.4](https://www.infineon.com/cms/en/design-support/tools/sdk/psoc-software/psoc-creator/) in the build configuration Release.
- A compiled image (KitProg3_1.hex and KitProg3_1.cyacd) will be placed in the /KitProg3.cydsn/ARM_GCC_541/Release folder.
- There are two options how to programm recently compiled images to PSoC5LP on the Cypress kit:
  - KitProg3_1.hex can be programmed with an external debug probe.
  - KitProg3_1.cyacd can be programmed using the fw-loader utility:
     1. Copy the KitProg3_1.cyacd file to <install folder fw-loader utility>/../kp-firmware/KitProg3.cyacd in the fw-loader utility.
     2. Use the --update-kp3 option of the fw-loader utility to update the firmware.

### Notes
Note, this KitProg3 firmware source code is without the DapLink code part, which is stored in the separate GitHub repo - https://github.com/ARMmbed/DAPLink . We are working with the ARM team to integrate our updates into this repo.

### More information

-   [fw-loader README.md](https://github.com/Infineon/Firmware-loader/blob/master/README.md)

-   [Kitprog3 User Guide](https://www.infineon.com/documentation/development-kitsboards/kitprog-user-guide)

-   [KitProg Host Protocol Interface](https://www.infineon.com/dgdl/Infineon-KitProg_Host_Protocol_Interface-UserManual-v01_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0f0125c8185e)

-   [ModusToolbox™ Software Environment, Quick Start Guide, Documentation, and
    Videos](https://www.infineon.com/modustoolbox)

-   [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)

© Cypress Semiconductor Corporation, 2019-2024. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").
