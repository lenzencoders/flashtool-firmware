# LENZ FlashTool Firmware

Main firmware for the LENZ FlashTool – a USB programmer, calibrator, and angle data reader for LENZ encoders.

## Hardware & Bootloader Links

- **Hardware (PCB) Repository:** [https://oshwlab.com/lenz-encoders/usb_biss_c](https://oshwlab.com/lenz-encoders/usb_biss_c)
- **Bootloader Repository:** [https://github.com/lenzencoders/lenz-flashtool-bootloader](https://github.com/lenzencoders/lenz-flashtool-bootloader)

## Building and Flashing Guide

This guide covers everything from cloning the repository with submodules to generating the final firmware and flashing it to the FlashTool device.

### Prerequisites

- **Git** (with submodule support)
- **Python 3.8+**
- **ST-Link** programmer (for bootloader flashing)
- **OpenOCD** or **st-tools** (st-flash)
- Basic familiarity with command line

### 1. Clone the Repository with Submodules

The firmware repository includes the BiSS C communication library as a Git submodule.

```bash
git clone --recurse-submodules https://github.com/lenzencoders/flashtool-firmware.git
cd flashtool-firmware
```
If you've already cloned without **--recurse-submodules**:
```bash
git submodule init
git submodule update --recursive
```

### 2. Hardware Connections (SWD Pinout)
The FlashTool board features a 6-pin SWD connector for programming and debugging. Connect your ST-Link programmer as follows:

| ST-Link Pin   | FlashTool SWD Connector | Pin Number |
| ------------- |:-----------------------:| ----------:|
| 3.3V          | 3V3                     | Pin 1      |
| NRST (Reset)  | NRST                    | Pin 2      |
| GND           | GND2                    | Pin 4      |
| SWDIO         | SWD                     | Pin 5      |
| SWCLK         | SWC                     | Pin 6      |

### 3. Generate the Application Firmware
You need two input hex files:

- `firmware_FT_1.0.12.hex` (from this repository's release)

- `bootloader_FT_ver_1_0_3.hex` (from the bootloader repository)

**Clone the helper library**:
```bash
git clone https://github.com/lenzencoders/lenz-flashtool-lib.git
cd lenz-flashtool-lib
```
**Generate the combined firmware**:
```python
from lenz_flashtool.flashtool.hex_generator import generate_hex_main_fw
generate_hex_main_fw('firmware_FT_1.0.12.hex', 'bootloader_FT_ver_1_0_3.hex')
```
This produces `app_1.0.12.hex` – the complete application firmware with CRC checksums and metadata.

### 4. Flash the Bootloader (First Time / Recovery)
If your device doesn't have a bootloader yet, flash it using ST-Link:

**Using st-flash**:

```bash
st-flash --connect-under-reset --reset write bootloader_FT_ver_1_0_3.hex 0x8000000
```
**Using OpenOCD**:

```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
        -c "program bootloader_FT_ver_1_0_3.hex verify reset exit"
```

### 5. Flash the Application Firmware
Once the bootloader is present and `app_1.0.12.hex` is generated, use the `download_fw_to_ft` function from `lenz-flashtool-lib`:

```bash
cd lenz-flashtool-lib
```
**Download the combined firmware**:
```python
import logging
import lenz_flashtool as lenz

lenz.init_logging('flashtool.log', logging.INFO, logging.DEBUG)
with lenz.FlashTool(port_description_prefixes=('XR21V')) as ft:
    ft.download_fw_to_ft('app_1.0.12.hex', max_retries=3, pbar=True)
```
Note: Ensure the device is in bootloader mode.

### Complete Workflow Example
```bash
# 1. Clone all repositories
git clone --recurse-submodules https://github.com/lenzencoders/flashtool-firmware.git
git clone https://github.com/lenzencoders/flashtool-bootloader.git
git clone https://github.com/lenzencoders/lenz-flashtool-lib.git

# 2. Generate combined firmware
cd lenz-flashtool-lib
python -c "
from hex_generator import generate_hex_main_fw
generate_hex_main_fw(
    '../flashtool-firmware/firmware_FT_1.0.12.hex',
    '../flashtool-bootloader/bootloader_FT_ver_1_0_3.hex'
)
"

# 3. Flash bootloader (first time only)
cd ../flashtool-bootloader
st-flash write bootloader_FT_ver_1_0_3.hex 0x8000000

# 4. Flash application via bootloader
cd ../lenz-flashtool-lib
python -c "from core import FlashToolUpdater; FlashToolUpdater().download_fw_to_ft('app_1.0.12.hex')"
```