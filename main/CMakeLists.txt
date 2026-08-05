idf_component_register(
    SRCS
        "main.c"
        "modbus.c"
        "solar.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_pm esp_timer nvs_flash
)