# SENSOR - Aplhasense Co-AE and Co-AF Carbon Monoxide Sensor with PSNERGY ADUcm355 electrochemical board
# VENDOR - PSNERGY
# DATASHEET - ADuCM355
# INTERFACE - I2C
# DOCS - 
# MODULE PLATFORM - CircuitPython

import time
import machine
import json

param_addrs = {
    "get_status"                 :  0x02,
    "set_temperature"            :  0x03,
    "read_temperature"           :  0x04,
    "set_humidity"               :  0x05,
    "read_humidity"              :  0x06,
    "set_meas_time_ms"           :  0x07,
    "read_meas_time_ms"          :  0x53,
    "start_measurements"         :  0x08,
    "stop_measurements"          :  0x0a,
    "set_tia_gain"               :  0x0c,
    "read_tia_gain"              :  0x0e,
    "set_vbias"                  :  0x10,
    "read_vbias"                 :  0x12,
    "set_vzero_voltage"          :  0x14,
    "read_vzero_voltage"         :  0x16,
    "set_sensor_type"            :  0x18,
    "set_sensor_sens"            :  0x19,
    "read_sensor_type"           :  0x1a,
    "read_sensor_sens"           :  0x1b,
    "run_eis"                    :  0x1e,
    "read_eis_results"           :  0x1f,
    "read_avg_lsb"               :  0x20,
    "read_eis_results_full"      :  0x25,
    "read_raw_lsb"               :  0x26,
    "read_avg_ppb"               :  0x30,
    "read_raw_ppb"               :  0x40,
    "run_pulse_test"             :  0x45,
    "read_pulse_test_results"    :  0x50,
    "set_rload"                  :  0x51,
    "read_rload"                 :  0x52,
    "read_200r_rtia_cal_result"  :  0x55,
    "set_i2c_address"            :  0x80,
}

param_lens = {
    "get_status"                 : 1,
    "set_temperature"            : 2,
    "read_temperature"           : 2,
    "set_humidity"               : 2,
    "read_humidity"              : 2,
    "set_meas_time_ms"           : 2,
    "read_meas_time_ms"          : 2,
    "start_measurements"         : 1,
    "stop_measurements"          : 1,
    "set_tia_gain"               : 1,
    "read_tia_gain"              : 1,
    "set_vbias"                  : 2,
    "read_vbias"                 : 2,
    "set_vzero_voltage"          : 2,
    "read_vzero_voltage"         : 2,
    "set_sensor_type"            : 1,
    "set_sensor_sens"            : 3,
    "read_sensor_sens"           : 3,
    "read_sensor_type"           : 1,
    "run_eis"                    : 2,
    "read_eis_results"           : 1,
    "read_avg_lsb"               : 2,
    "read_eis_results_full"      : 1,
    "read_raw_lsb"               : 2,
    "read_avg_ppb"               : 3,
    "read_raw_ppb"               : 3,
    "run_pulse_test"             : 3,
    "read_pulse_test_results"    : 1,
    "set_rload"                  : 1,
    "read_rload"                 : 1,
    "read_200r_rtia_cal_result"  : 1,
    "set_i2c_address"            : 1,
}

# if a param uses two's complement, list it's max range here, the read_param function will check and scale accordingly
param_max_val = {
    "read_raw_ppb"               : 2**24,
    "read_avg_ppb"               : 2**24,
    "read_counts"                : 2**16,
}

jfile = open("AnalyzerConfig.json")
Configuration = json.load(jfile)

class Electrochem():
    def __init__(self, addr, bus, sensor_type=1):
        self.addr = addr
        self.bus = bus
        

        sts  = int(self.read_param("get_status"))
        while sts != 1:
            print(sts)
            print("Trying to connect to EC sensor...")
            sts  = int(self.read_param("get_status"))
        print("\n+------------ EC Sensor Connected! -------------+\n")
        try:
            d2 = self.read_param("read_vzero_voltage")
            d3 = self.read_param("read_sensor_sens")
            d4 = self.read_param("read_tia_gain")
            d5 = self.read_param("read_rload")
        except Exception as e:
            print("Cannot initially read values: ", e)
        try:
            if(d2 != 1100):
                self.write_param("set_vzero_voltage", 1100)
            if(d3 != 7000):
                self.write_param("set_sensor_sens", 7000)
            if(d4 != 1):
                self.write_param("set_tia_gain", 1)
            if(d5 != 1):
                self.write_param("set_rload", 1)
        except Exception as e:
            print("Cannot initialize EC Values")

        


    def configure_sensor(self, vbias, vzero_voltage, sensor_sens, tia_gain, rload):
        while True:
            try:
                self.vbias = int(vbias)
                self.vzero_voltage = int(vzero_voltage)
                self.sensor_sens = int(sensor_sens)
                self.tia_gain = int(tia_gain)
                self.rload = int(rload)
                
                self.write_param("set_vbias", self.vbias)
                time.sleep(0.01)
                self.write_param("set_vzero_voltage", self.vzero_voltage)
                time.sleep(0.01)
                self.write_param("set_sensor_sens", self.sensor_sens)
                time.sleep(0.01)
                self.write_param("set_tia_gain", self.tia_gain)
                time.sleep(0.01)
                self.write_param("set_rload", self.rload)
                time.sleep(0.01)

                check = 0
                if self.read_param("read_vbias") == self.vbias: check+=1
                if self.read_param("read_vzero_voltage") == self.vzero_voltage: check+=1
                if self.read_param("read_sensor_sens") == self.sensor_sens: check+=1
                if self.read_param("read_tia_gain") == self.tia_gain: check+=1
                if self.read_param("read_rload") == self.rload: check+=1
                if check == 4:
                    break
                else:
                    print("EC config writing error: config does not match", check)
                    raise(Exception)

            except Exception as e:
                time.sleep(0.5)
                print(e)



    def setCalValues(self, raw_Low, raw_High, ref_Low, ref_High):
        self.raw_Low = raw_Low
        self.raw_High = raw_High
        self.ref_Low = ref_Low
        self.ref_High = ref_High
        
    def get_param_len(self, param):
        return param_lens[param]
    def get_param_addr(self, param):
        return param_addrs[param]
    def get_param_max_val(self, param):
        if param in param_max_val:
            return param_max_val[param]
        else:
            return(-1)
    def get_params():
        return param_addrs.keys()

    def read_param(self, param):
        val = None
        param_len = self.get_param_len(param)
        param_addr = self.get_param_addr(param)
        ctr = 0

        for attempt in range(3):
            while True:

                data = bytearray(param_len)

                machine.I2C(2).writeto(self.addr, bytearray([param_addr]))
                machine.I2C(2).readfrom_into(self.addr, data)
                machine.I2C(2).readfrom_into(self.addr, data)
                machine.I2C(2).readfrom_into(self.addr, data)

                last_byte = (param_len-1)
                first_byte = (param_len - param_len)

                #if data[last_byte] is not 0xff or data[first_byte] is not 0xff
                if data[last_byte] is not 0xff:
                    val = 0
                    for x in range(0, param_len):
                        val |= (data[x]<<((param_len-x-1)*8))
                else:
                    time.sleep(0.01)
                    continue

                # check for two's completment if the param is in the list of params that are defined to have it
                max_val = self.get_param_max_val(param)
                if max_val != -1:
                    max_positive_val = (max_val / 2) - 1
                    if val > max_positive_val:
                        val = max_val - val
                        val *= -1
                    
                break
            attempt += 1
        return val


    def write_param(self, param, value):
        param_len = self.get_param_len(param)
        param_addr = self.get_param_addr(param)
        data = bytearray(param_len+1)
        data[0] = param_addr
        for x in range(1, param_len+1):
            rshift = (param_len-x)*8
            if rshift == 0:
                data[x] = (value&0xff)
            else:
                data[x] = (value>>rshift)

        for attempt in range(3):

            sts = machine.I2C(2).writeto(self.addr, data)

            attempt += 1
        return sts

    def readGas(self):
        try:
            self.avg_ppb = self.read_param("read_raw_ppb")
            self.avg_ppm = round(self.avg_ppb/1000)
            self.raw_ppb = self.avg_ppb
            self.raw_ppm = round(self.raw_ppb/1000)
            
            self.avgPPM = round((((self.avg_ppm - self.raw_Low) * (self.ref_High - self.ref_Low)) / (self.raw_High - self.raw_Low)) + self.ref_Low)

            if self.avgPPM < 10:
                self.avgPPM = 0
            
            self.CO_unfiltered = int(self.raw_ppm)

            return self.avgPPM

        except Exception as e:
            self.CO_filtered=-99
            print(e)

class CO(Electrochem):
    def __init__(self, addr, bus, sensor_type=1):
        super().__init__(addr, bus, sensor_type=1)
        self.vbias = 0
        self.raw_Low = int(Configuration["Electrochem"]["CO"]["raw_Low"])
        self.raw_high = int(Configuration["Electrochem"]["CO"]["raw_High"])
        self.ref_Low = int(Configuration["Electrochem"]["CO"]["ref_Low"])
        self.ref_High = int(Configuration["Electrochem"]["CO"]["ref_High"])
        vb = self.read_param("read_vbias")
        count = 0
        while vb != 0:
            self.write_param("set_vbias", self.vbias)
            count += 1
            vb = self.read_param("read_vbias")
            if count == 10:
                print("error initializing CO vbias")
                break
        


class NO(Electrochem):
    def __init__(self, addr, bus, sensor_type=1):
        super().__init__(addr, bus, sensor_type=1)
        self.vbias = 300
        self.raw_Low = int(Configuration["Electrochem"]["NO"]["raw_Low"])
        self.raw_high = int(Configuration["Electrochem"]["NO"]["raw_High"])
        self.ref_Low = int(Configuration["Electrochem"]["NO"]["ref_Low"])
        self.ref_High = int(Configuration["Electrochem"]["NO"]["ref_High"])
        vb = self.read_param("read_vbias")
        count = 0
        while vb != 300:
            self.write_param("set_vbias", self.vbias)
            count += 1
            vb = self.read_param("read_vbias")
            print(vb)
            if count == 10:
                print("error initializing NO vbias")
                break
        
