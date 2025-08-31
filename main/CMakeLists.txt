idf_component_register(SRCS "ble_keyboard.c" "led_driver.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                            "hid_host.c"
                    PRIV_REQUIRES bt usb nvs_flash esp_driver_gpio esp_driver_rmt
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
