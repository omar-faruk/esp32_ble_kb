# usb-ble-kb

Firmware to turn a standard USB keyboard into a Bluetooth (BLE HID) keyboard using an ESP32-series chip.

Short: plug a USB keyboard into the ESP device (USB-host capable board), the firmware reads standard USB HID keyboard reports and forwards them as a BLE HID keyboard to a paired host (PC, phone, tablet).

## Features
- USB HID Host keyboard parsing
- BLE HID Device (keyboard) advertising and pairing
- LED control (caps/num lock) support
- Configurable via ESP-IDF menuconfig / sdkconfig

## Quick build & flash (ESP-IDF)
1. Install and source ESP-IDF (see https://docs.espressif.com).
2. From project root:
   - Set target / select sdkconfig as needed (examples provided: `sdkconfig.defaults.esp32c2`, `sdkconfig.defaults.esp32c3`, `sdkconfig.defaults.esp32s3`)
   - Build and flash:
     ```sh
     idf.py build
     idf.py -p /dev/ttyUSB0 flash monitor
     ```
   - Use `idf.py menuconfig` to tune BLE name, HID parameters, and USB host settings.

## Hardware / wiring
- Use an ESP board with USB-host capability (or an external USB host shield).
- Connect the keyboard's USB (Type-A -> USB host port). Provide sufficient power for the keyboard.
- Ensure USB D+/D- and VBUS are correctly connected according to your board.

## Project layout
- [ble_keyboard.c](http://_vscodecontentref_/0) — main firmware logic and entrypoint (main/ble_keyboard.c)
- [esp_hidd_prf_api.c](http://_vscodecontentref_/1) / esp_hidd_prf_api.h — BLE HID profile glue (main/esp_hidd_prf_api.c, main/esp_hidd_prf_api.h)
- [hid_dev.c](http://_vscodecontentref_/2) / hid_dev.h — USB HID parsing and device abstraction (main/hid_dev.c, main/hid_dev.h)
- [hid_device_le_prf.c](http://_vscodecontentref_/3) — BLE HID device profile implementation (main/hid_device_le_prf.c)
- [hid_host.c](http://_vscodecontentref_/4) / hid_host.h — USB HID host-side handling (main/hid_host.c, main/hid_host.h)
- [hid_usage_keyboard.h](http://_vscodecontentref_/5) — HID usages for keyboard (main/hid_usage_keyboard.h)
- [hid_usage_mouse.h](http://_vscodecontentref_/6) — HID usages for mouse (if referenced) (main/hid_usage_mouse.h)
- [hid.h](http://_vscodecontentref_/7) — core HID definitions (main/hid.h)
- [hidd_le_prf_int.h](http://_vscodecontentref_/8) — internal HID profile defs (main/hidd_le_prf_int.h)
- [led_driver.c](http://_vscodecontentref_/9) / led_driver.h — LED indicators (Caps/Num) (main/led_driver.c, main/led_driver.h)
- [CMakeLists.txt](http://_vscodecontentref_/10) (project) — build configuration (CMakeLists.txt)
- [CMakeLists.txt](http://_vscodecontentref_/11) — main component build rules (main/CMakeLists.txt)

## Useful files in this workspace
- [.clangd](http://_vscodecontentref_/12)
- [.gitignore](http://_vscodecontentref_/13)
- [CMakeLists.txt](http://_vscodecontentref_/14)
- [README.md](http://_vscodecontentref_/15)
- [LICENSE](http://_vscodecontentref_/16)
- [sdkconfig](http://_vscodecontentref_/17)
- [sdkconfig.defaults](http://_vscodecontentref_/18)
- [sdkconfig.defaults.esp32c2](http://_vscodecontentref_/19)
- [sdkconfig.defaults.esp32c3](http://_vscodecontentref_/20)
- [sdkconfig.defaults.esp32s3](http://_vscodecontentref_/21)
- [devcontainer.json](http://_vscodecontentref_/22)
- [Dockerfile](http://_vscodecontentref_/23)
- [c_cpp_properties.json](http://_vscodecontentref_/24)
- [launch.json](http://_vscodecontentref_/25)
- [settings.json](http://_vscodecontentref_/26)
- [ble_keyboard.c](http://_vscodecontentref_/27)
- [CMakeLists.txt](http://_vscodecontentref_/28)
- [esp_hidd_prf_api.c](http://_vscodecontentref_/29)
- [esp_hidd_prf_api.h](http://_vscodecontentref_/30)
- [hid_dev.c](http://_vscodecontentref_/31)
- [hid_dev.h](http://_vscodecontentref_/32)
- [hid_device_le_prf.c](http://_vscodecontentref_/33)
- [hid_host.c](http://_vscodecontentref_/34)
- [hid_host.h](http://_vscodecontentref_/35)
- [hid_usage_keyboard.h](http://_vscodecontentref_/36)
- [hid_usage_mouse.h](http://_vscodecontentref_/37)
- [hid.h](http://_vscodecontentref_/38)
- [hidd_le_prf_int.h](http://_vscodecontentref_/39)
- [led_driver.c](http://_vscodecontentref_/40)
- [led_driver.h](http://_vscodecontentref_/41)

## Troubleshooting
- If keyboard not detected: verify USB power and host port wiring.
- If BLE not visible: check BLE advertising in menuconfig and ensure controller enabled.
- Use `idf.py monitor` logs to inspect USB/HID and BLE stack messages.

## License
This project includes a GPLv3 license file: [LICENSE](http://_vscodecontentref_/42).

## Contact
- Preferred: open an issue in this repository.
- Email: omarfaruk.dc@gmail.com
