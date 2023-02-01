# SENSOR - Honeywell TruStability SSC Series
# VENDOR - https://sensing.honeywell.com/sensors/amplified-board-mount-pressure-sensors
# DATASHEET - https://sensing.honeywell.com/honeywell-sensing-trustability-ssc-series-standard-accuracy-board-mount-pressure-sensors-50099533-a-en.pdf
# INTERFACE - i2c
# DOCS - https://sensing.honeywell.com/i2c-comms-digital-output-pressure-sensors-tn-008201-3-en-final-30may12.pdf
# MODULE PLATFORM - CircuitPython


import time
import machine

class SSC:

    def __init__(self, addr, bus, Pmin, Pmax):
        self.device = addr
        self.Pmin = Pmin
        self.addr = addr
        self.Pmax = Pmax
        self.outMin = 14745
        self.outMax = 1638
        self.bus = bus
        self.lastVal = None

        # set default unit
        self.change_unit("inh2o")
        self.zero_offset = 0

        self.get()
        pass

    def get(self):
        try:
            block = bytearray(2)
            machine.I2C(self.bus).readfrom_into(self.addr, block)
            self.digitalcount = (((block[0] & 0b00111111) << 8) | block[1])
            self.pressure = ((((self.digitalcount-self.outMin)*(self.Pmax-self.Pmin))/(self.outMax-self.outMin))+self.Pmin)
            self.pressure = round((self.pressure + self.zero_offset), 4)
            self.lastVal = self.pressure
            return(self.pressure)
        except Exception as e:
            print("Pressure Sensor Pressure Read Error", e)
            return lastVal

    def change_unit(self, unit):
        if unit == "inh2o":
            self.unit = "inh2o"
            self.conversion_multiplier = 1.0
            self.rounder = 1
        elif unit == "mbar":
            self.unit = "mbar"
            self.conversion_multiplier = 2.4884
            self.rounder = 1
        elif unit == "kpa":
            self.unit ="kPa"
            self.conversion_multiplier = 0.24884
            self.rounder = 1
        elif unit == "oz-in^2":
            self.unit = "oz-in^2"
            self.conversion_multiplier = 0.577466
            self.rounder = 2
        elif unit == "psi":
            self.unit = "PSI"
            self.conversion_multiplier = 0.036091
            self.rounder = 2
        else:
            print("Failed to change unit when unit = ", unit)

    def zero(self):
        self.zero_offset = 0
        temp_unit = self.unit
        self.change_unit("PSI")
        i = 0
        zero = []
        while i < 20:
            zero.append(self.get())
            i += 1
        avg = sum(zero)/len(zero)
        self.zero_offset = 0 - avg
        self.change_unit(temp_unit)

    def check_zero(self):
        temp_offset = self.zero_offset
        self.zero_offset = 0
        temp_unit = self.unit
        self.change_unit("PSI")
        i = 0
        zero = []
        while i < 20:
            zero.append(self.get())
            time.sleep(.001)
            i += 1
        avg = sum(zero)/len(zero)
        self.check_zero = 0 - avg
        self.change_unit(temp_unit)
        self.zero_offset = temp_offset
        print("Check offset returned: ", round(self.check_zero,4))

    def get_temp(self):
        # get the temperature from the pressure sensor, would have to read 4 bytes not 2
        try:
            block = bytearray(4)
            machine.I2C(self.bus).readfrom_into(self.addr, block)
            temp = block[2] << 3 | ((block[3] >> 5) & 0b00000111)
            self.temperature = round((((temp/2047)*200)-50),1)
            return(self.temperature)
        except OSError as e:
            print("Pressure Sensor Temp Read Error", e)
            #return last value
            return(self.temperature)