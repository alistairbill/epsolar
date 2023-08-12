idf_build_get_property(target IDF_TARGET)
set(srcs "common.c" "main.c" "modbus.c" "solar.c")

if (IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs "connect_esp32.c")
else()
    list(APPEND srcs "connect_esp8266.c")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")