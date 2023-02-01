# SENSOR - SST Luminox Luminox Optical Oxygen Sensor
# VENDOR - https://www.sstsensing.com/product/luminox-optical-oxygen-sensors-2/
# DATASHEET - https://www.sstsensing.com/wp-content/uploads/2017/07/DS0030rev13_LuminOx.pdf
# INTERFACE - UART
# DOCS - https://www.sstsensing.com/wp-content/uploads/2018/01/UG-001_LuminOx_UserGuide_rev2.pdf
# MODULE PLATFORM - CircuitPython


import time
import machine
LAST_ATTEMPT_TIME = time.ticks_ms()

class Luminox:

    def __init__(self, uart_device, o2_offset):
        self.device = machine.UART(5, 9600)
        self.device.readline()
        self.o2_offset = o2_offset

        alive = False
        while alive is False:
            try:
                self.device.write(b'M 1\r\n')
                time.sleep(.1)
                response = self.device.readline()
                if response == b'M 01\r\n':
                    pass             


                time.sleep(.1)
                

                check_dict = {
                    "1 Date of Manufacture: ": '# 0',
                    "2 Serial Number: ": '# 1',
                    "3 Software Revision: ": '# 2',
                    "4 First Reading: ": 'A '
                }
                ans_list = ['_____O2 DIAGNOSTIC INFORMATION_____']

                for i in check_dict:
                    msg = '{}\r\n'.format(check_dict[i])
                    self.device.write(bytes(msg, 'utf-8'))
                    ans = self.device.readline()
                    ans = ans.decode().rstrip('\r\n')
                    ans_list.append((i +ans))
                    time.sleep(.1)

                #this isn't going to print out in order.. decause dictionaries, but whatever idc rn
                print(*ans_list, sep='\n')
                self.get()
                alive = True
            except AttributeError as e:
                print("Luminox not responding... retrying in 1s", e)
                time.sleep(1)

    def identify(self):
        #untested
        try:
            self.device.write(b'# 0\r\n')
            raw = self.device.readline()
            if raw is not None:
                if len(raw) == 15:
                    raw = raw.rsplit()
                    self.id_birthday_year = raw[1]
                    self.id_birthday_day = raw[2]

            self.device.write(b'# 1\r\n')
            raw = self.device.readline()
            if raw is not None:
                if len(raw) == 14:
                    raw = raw.rsplit()
                    self.id_serial_number = str(raw[1] + raw[2])

            self.device.write(b'# 2\r\n')
            raw = self.device.readline()
            if raw is not None:
                if len(raw) == 9:
                    raw = raw.rsplit()
                    self.id_software_revision = raw[1]
        except Exception as e:
            print(e)


    def get(self):
        global LAST_ATTEMPT_TIME
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, LAST_ATTEMPT_TIME) > 500:
            LAST_ATTEMPT_TIME = current_time
            for attempt in range(15):
                try:
                    # print('\t Actually attempting to read')
                    self.device.write(b'A\r\n')
                    raw_all = self.device.readline()
                    if raw_all is not None:
                        raw_all = raw_all.decode()
                        if len(raw_all) == 41:  
                            self.all_output = raw_all.rsplit()
                            self.ppo2 = float(self.all_output[1])
                            self.temp = float(self.all_output[3])
                            self.pressure = float(self.all_output[5])
                            # old
                            # self.o2 = float(self.all_output[7])
                            # new
                            self.o2 = float(self.all_output[7])+self.o2_offset
                            if self.o2 < 0:
                                self.o2 = 0
                            self.errors = int(self.all_output[9])
                            self.analyzer()
                            LAST_ATTEMPT_TIME = time.ticks_ms()
                            break
                    if (raw_all is None) or (len(raw_all) is not 41):
                        raise ValueError('Luminox Read Error')
                except Exception as e:
                    time.sleep(.1)
                    print(e, "Attempt", attempt)
                else:
                    raise AttributeError('Mega O2 Error')
        else:
            # keep the graph moving at the same pace as pressure
            return(self.o2)
        # https://stackoverflow.com/questions/2083987/how-to-retry-after-exception


    def analyzer(self):
        # in theory someday this could analyze all local data and set flags high accordingly
        if self.o2 < 1.9:
            self.flag_o2_CRIT = True
        if self.errors:
            pass
        if self.temp:
            pass