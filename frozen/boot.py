from pyb import Pin
import pyb

en3v3 = Pin('A15', Pin.OUT_PP, pull=Pin.PULL_DOWN)
en3v3.value(0)

en5 = Pin('C9', Pin.OUT_PP, pull=Pin.PULL_DOWN)
en5.value(0)

pyb.country('US') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
pyb.usb_mode('VCP') # act as a serial device only
