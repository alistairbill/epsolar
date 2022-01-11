# This file is executed on every boot (including wake-boot from deepsleep)
import esp
esp.osdebug(None)
import uos
uos.dupterm(None, 1) # disable REPL on UART(0)
import gc
from machine import RTC
try:
    if RTC().memory() == b"webrepl":
        raise

    import main
    from homie.network import disable_ap
    disable_ap()
except Exception:
    import settings
    import webrepl
    from network import WLAN, STA_IF
    wlan = WLAN(STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(settings.WIFI_SSID, settings.WIFI_PASSWORD)
        while not wlan.isconnected():
            pass
    webrepl.start(password="uhomie")
gc.collect()