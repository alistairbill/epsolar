# Modules
freeze(
    "$(PORT_DIR)/modules",
    (
        "_boot.py",
        "apa102.py",
        "flashbdev.py",
        "inisetup.py",
        "ntptime.py",
        "port_diag.py",
    ),
)

# Tools
# freeze("$(MPY_DIR)/tools", ("upip.py", "upip_utarfile.py"))

# Microhomie
freeze(
    ".",
    (
        "homie/__init__.py",
        "homie/constants.py",
        "homie/device.py",
        "homie/network.py",
        "homie/node.py",
        "homie/property.py",
        "homie/validator.py",
    ),
)

# uModbus
# freeze(
#     ".",
#     (
#         "uModbus/functions.py",
#         "uModbus/serial.py",
#     ),
# )

# Libs
freeze("./lib", (
    "mqtt_as.py",
    "primitives/__init__.py"
    )
)

# uasyncio
include("$(MPY_DIR)/extmod/uasyncio/manifest.py")

# webrepl
include("$(MPY_DIR)/extmod/webrepl/manifest.py")
