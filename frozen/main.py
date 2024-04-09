import time, machine, pyb, usys, stm, json, gc
from machine import Pin, I2C, UART
from pyb import Pin, LED
from FrameGen import frame

pyb.freq(168000000,168000000,42000000,84000000)

# BOOT UP
AWAKE_START = time.ticks_ms()
button = Pin('A1', Pin.IN, pull=Pin.PULL_DOWN)
en5 = Pin('C9', Pin.OUT_PP)
en5.value(0)
en3v3 = Pin('A15', Pin.OUT_PP)
en3v3.value(0)
tag = Pin('B1', Pin.IN, pull=Pin.PULL_DOWN)
xbee = frame(2, 115200)

def check_button(b):
    a = 0
    start_time = time.ticks_ms()
    hold_ms = 550
    check_int_ms = 10
    elapsed = 0
    debounce_pct = 0.80
    while elapsed < hold_ms:
        if b.value() == 1:
            a += 1
        time.sleep_ms(check_int_ms)
        elapsed = time.ticks_diff(time.ticks_ms(), start_time)
    if a > (hold_ms/check_int_ms)*debounce_pct:
        print("good button press... booting")
        return("boot")
    else:
        print("bad button press... sleeping")
        return("sleep")

while True:
    if button.value() ==1:
        if check_button(button) == 'boot':
            en3v3.value(1)
            en5.value(1)
            break
    elif tag.value() == 1:
        en3v3.on()
        en5.on()
        machine.UART(2).deinit()
        machine.UART(6).deinit()
        continue
    else:
        if time.ticks_diff(time.ticks_ms(), AWAKE_START) < 10000:
            pass
        else:
            stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
            machine.deepsleep()

time.sleep(0.1)
en5.value(1)
en3v3.value(1)

# INITIAL CONFIG OF ANALYZER
import PRS_TruStabilitySSC as PRS_TruStabilitySSC
from __init__ import PSNERGY_Manometer as Screen
import measurements as measurements
from MAX17263 import MAX17263

jfile = open("AnalyzerConfig.json")
Configuration = json.load(jfile)
jfile.close()
print("Serial Number: ", Configuration["DeviceInfo"]["SN"])
print("MCU Version: ", Configuration["DeviceInfo"]["mcu_version"])
print("Version Number: ", Configuration["DeviceInfo"]["VN"])
DEVICE_ID = Configuration["XbeeConfig"]["deviceID"]
i2c_bus = I2C(2)
i2c_bus1 = I2C(1)
COORDINATOR_ADDR = "0000000000000000"
time.sleep(0.1)


#screen setup
try:
    nextion = machine.UART(6, 115200)
    nextion.init(baudrate=115200, bits=8, parity=None, stop=1, timeout_char=1, timeout=3)
    screen = Screen(nextion, debug_mode=False)
    time.sleep(0.5)
    screen.set_device_name(str(DEVICE_ID))
    screen.set_signal_strength(1)
    screen.boot_progress("Initializing Screen...", 20)
except Exception as e:
    print("Screen Initialization error: ", e)
    screen.boot_progress("Screen Initialization Failure!", 20)
time.sleep(1)

#battery setup
try:
    battery = MAX17263(0x36, i2c_bus1, 0.01)
    screen.boot_progress("Initializing Battery...", 30)
    if battery.oc_voltage() <= 3.0:
        screen.change_page("deadBatt")
        time.sleep(10)
        en3v3.value(0)
        en5.value(0)
        time.sleep(3)
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
        machine.deepsleep()
    time.sleep(0.01)
except Exception as e:
    print("Battery setup failure: ", e)
    screen.boot_progress("Battery Initialization Failure!", 30)
time.sleep(0.1)

try:
    screen.set_battery_percent(int(battery.batt_percent()))
except Exception as e:
    print(e)

#pressure setup
try:
    screen.boot_progress("Initializing Pressure Sensor...", 50)
    while True:
        time.sleep(0.05)
        prs_sensor = PRS_TruStabilitySSC.SSC(0x28, 2, -27.7076, 27.7076)
        pressure = measurements.Measurement()
        pressure.create_stream(100, 4)
        break
except Exception as e:
    try:
        while True:
            time.sleep(0.05)
            prs_sensor = PRS_TruStabilitySSC.SSC(0x38, 2, -27.7076, 27.7076)
            break
        pressure = measurements.Measurement()
        pressure.create_stream(100, 4)
    except Exception as e:
        print("Pressure Initialization Failure: ", e)
        screen.boot_progress("Pressure Sensor Initialization Failure!", 50)

time.sleep(0.01)
        
#xbee setup
try:
    screen.boot_progress("Initializing XBee...", 80)
    xbee = frame(2, 115200)
    xbeeID = DEVICE_ID
except Exception as e:
    print("Xbee Initialization Failure: ", e)

time.sleep(0.01)

def cmd_handler():
    global screen, pressure, prs_sensor, prsUnits, location, save, cal, cancel, network, msg
    global saveSend, health, healthSend, p_inh2o, xbee, first, newCal, temporary, locationSend, bigNet, netTime
    instr = None
    xb = None
    state = None
    try:
        instr = screen.read_instr()
        if instr != "" and instr != None and instr != '' and instr != "b''":
            print("Instr: ", instr)
            if instr[0] == "moving":
                pressure.stream_size = int(instr[1])
                temporary = pressure.stream_size
            elif instr[0] == "unit":
                prs_sensor.change_unit(prsUnits[int(instr[1])])
            elif instr[0] == "location":
                location = instr[1:]
                locationSend = True
            elif instr[0] == "save":    # "save flue initial", "save pressure final"
                if instr[1] == "pressure":
                    save[0] = "pressure"
                    save[1] = str(pressure.average())
                    save[2] = str(instr[2])
                saveSend = True
            elif instr[0] == "report":
                health = instr[1:]
                healthSend = True
            elif instr[0] == "zero":
                loadCalibration()
                prs_sensor.zero()
                writeCalVals()
                newCal = True
            elif instr[0] == "cancel":
                cancel = True
            elif instr[0] == "dnd":
                screen.notifications(bool(instr[1]))
            else:
                err = ("command not recognized: " + str(instr))
                print(err)

        if time.ticks_diff(time.ticks_ms(), first) > 5000:
            xb = xbee.xbRead()
            if xb != -1:
                if b'cmd' not in xb:
                    dic = xbee.receiveFrame(xb)
                    if dic != -1  and str(dic["Status"]) == "0x00" or str(dic["Status"]) == "0x0":
                        network = True
                        screen.set_signal_strength(2)
                        print("XBee Status: ", dic["Status"])
                    elif dic != -1 and str(dic["Status"]) != "0x00" and str(dic["Status"]) != "0x0":
                        network = False
                        screen.set_signal_strength(0)
                        print("XBee Status: ", dic["Status"])
                elif b'cmd' in xb and network is True:
                    print("XB: ", xb)
                    xb = xb[15:-1]
                    xb = str(xb)
                    xb = xb[2:-1]
                    xb = xb.split()
                    if xb[0] == "cmd" and len(xb) > 1:
                        print("---------------------------- ", xb)
                        if xb[1] == "shutdown":
                            shutdown()
                        elif xb[1] == "sleep":
                            screen.write_global("sleep", xb[2])
                        elif xb[1] == "save":    # "save flue initial", "save pressure final"
                            save[0] = "pressure"
                            save[1] = str(pressure.average())
                            if xb[2] == "initial" or xb[2] == "final":
                                save[2] = xb[2]
                            saveSend = True
                        elif xb[1] == "zero":
                            loadCalibration()
                            prs_sensor.zero()
                            writeCalVals()
                        elif xb[1] == "unit":
                            #psi, mbar, inh2o, kPa
                            if int(xb[2]) >= 0 and int(xb[2]) <= 3:
                                prs_sensor.change_unit(prsUnits[int(xb[2])])
                                screen.touch_component("{}{}".format("r", int(xb[2])+4))
                                screen.release_component("{}{}".format("r", int(xb[2])+4))
                        elif xb[1] == "moving":
                            pressure.stream_size = int(xb[2])
                        elif xb[1] == "magic":
                            screen.magic_number(str(xb[2]))
                        elif xb[1] == "location":
                            locationSend = True
                        elif xb[1] == "bigneton":
                            if int(xb[2]) > 1 and int(xb[2]) <= 30:
                                netTime = int(xb[2])*1000
                            else:
                                netTime = 1000
                            bigNet = True
                        elif xb[1] == "bignetoff":
                            bigNet = False
                            netTime = 1000
                        elif xb[1] == "battery_saver_on":
                            screen.change_page("general")
                            time.sleep(0.25)
                            screen.touch_component("btON")
                            time.sleep(0.25)
                            screen.release_component("btON")
                            time.sleep(0.25)
                        elif xb[1] == "battery_saver_off":
                            screen.write_global("sleep", 0)
                            time.sleep(0.25)
                            screen.change_page("general")
                            time.sleep(0.25)
                            screen.touch_component("btOFF")
                            time.sleep(0.25)
                            screen.release_component("btOFF")
                            time.sleep(0.25)
                        elif xb[1] == "home":
                            screen.change_page("manometer")
                            time.sleep(0.25)
                        else:
                            err = ("xb command not recognized: " + str(xb))
                            print(err)
            del xb
        del instr
    except TypeError as t:
        pass
    except Exception as e:
        err = "Instruction Reading Failure: " + str(e)
        print(err)
        try:
            if network is True:
                msg = '{} {}'.format("E", err)
                xbee.txData(COORDINATOR_ADDR, msg)
                time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

def shutdown():
    global button, en3v3, en5, msg, xbee, network, screen
    try:
        en3v3.value(0)
        en5.value(0)
        time.sleep(1)
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
        machine.deepsleep()
    except Exception as e:
        err = "Shutdown Error: " + str(e)
        print(err)
        screen.error(err)
        try:
            if network is True:
                msg = '{} {}'.format("E", err)
                xbee.txData(COORDINATOR_ADDR, msg)
                time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

def loadCalibration():  #load values from config text into local variables
    global prs_sensor, msg, xbee, network, screen
    try:
        config = list(range(1))
        iterator = 0
        chars = 0
        with open('ConfigValues.txt') as f:  #Loads the rest of the number values into config array
            config[0] = f.readline()
        f.close()
        prs_sensor.zero_offset = float(config[0])    
        return "Done loading"
    except Exception as e:
        err = "EC Calibration Load Error: " + str(e)
        print(err)
        screen.error(err)
        try:
            if network is True:
                msg = '{} {}'.format("E", err)
                xbee.txData(COORDINATOR_ADDR, msg)
                time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

def writeCalVals():
    global prs_sensor, msg, xbee, network, screen
    try:
        f = open('ConfigValues.txt', 'w')
        f.write(str(prs_sensor.zero_offset))
        f.close()
    except Exception as e:
        err = "Write Calibration Values to File Error: " + str(e)
        print(err)
        screen.error(err)
        try:
            if network is True:
                msg = '{} {}'.format("E", err)
                xbee.txData(COORDINATOR_ADDR, msg)
                time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

try:
    p_inh2o = p_unit = p_unit_noavg = current = percent = voltage = mcu_temp = ptemp = 0
    saveSend = cancel = lowPower = healthSend = locationSend = newCal = network = bigNet = sscError = batError = senseError = screenError = sendError = mainError = False
    netTime = 2000
    prsAvgNum = 3
    prsUnits = ["psi", "mbar", "inh2o", "kpa"]
    location = ["na", "na", "na", "na", "na", "na"]
    save = ["", "", ""]
    health = ["", "", "", "", "", ""]
    cal = r = msg = ""
    LED(4).on()
    last_health = last_normal = last_send = 0
    slope = 0
    startHum = 0
    x=0
    ctr = ctr2 = timedout = lastValue = 0
    temporary = pressure.stream_size
    loadCalibration()
    beginStartLoop = time.ticks_ms()
    screen.boot_progress("Boot Successful!", 100)
    time.sleep(0.1)
    screen.change_page("manometer")
    first = time.ticks_ms()
except Exception as e:
    err = "Init Variables Failure: " + str(e)
    print(err)
    screen.error(err)

while True:
    try:
        try:
            if newCal == True:
                loadCalibration()
                newCal = False
        except Exception as e:
            screen.error("Load Calibration Error: " + e)

        #===================================================== Sense Stuff =================================================================
        try:

            cmd_handler()

            try:
                p = prs_sensor.get()
                if p > -0.1 and p < 0.1:
                    p = 0.0
                pressure.add_point(p)
                p_inh2o = round(pressure.average(),3)
                p_unit = round(pressure.average()*prs_sensor.conversion_multiplier, 3)
                p_unit_noavg = round(p*prs_sensor.conversion_multiplier, 3)
                if abs(p_inh2o - lastValue) > 5:
                    slope+=1
                    if slope >= 100:
                        slope = 0
                        temporary = pressure.stream_size
                        pressure.stream_size = 1
                        p = prs_sensor.get()
                        if p > -0.1 and p < 0.1:
                            p = 0.0
                        pressure.add_point(p)
                        p_inh2o = round(pressure.average(), 3)
                        p_unit = round(pressure.average()*prs_sensor.conversion_multiplier, 3)
                        p_unit_noavg = round(p*prs_sensor.conversion_multiplier, 3)
                else:
                    slope=0
                    pressure.stream_size = temporary
                lastValue = p
                sscError = False
            except TypeError as t:
                pass
            except Exception as e:
                if sscError is False:
                    sscError = True
                    err = "SSC Error: " + str(e)
                    print(err)
                    screen.error(err)
                    try:
                        if network is True:
                            msg = '{} {}'.format("E", err)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.1)
                    except Exception as e:
                        print("Cannot send error packet: ", e)

            cmd_handler()

            try:
                voltage = battery.oc_voltage()
                current = battery.current_draw()
                percent = battery.batt_percent()
                bTemp = battery.batt_temp()
                screen.set_battery_percent(int(percent))
                if voltage <= 3.0:
                    shutdown()
                batError = False
            except TypeError as t:
                pass
            except Exception as e:
                if batError is False:
                    batError = True
                    err = "Battery Error: " + str(e)
                    print(err)
                    screen.error(err)
                    voltage = 5.0
                    current = -999
                    percent = 100
                    bTemp = 99
                    try:
                        if network is True:
                            msg = '{} {}'.format("E", err)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.1)
                    except Exception as e:
                        print("Cannot send error packet: ", e)

            cmd_handler()

            senseError = False

        except TypeError as t:
            pass
        except Exception as e:
            if senseError is False:
                senseError = True
                err = "Sense Error: " + str(e)
                print(err)
                screen.error(err)
                try:
                    if network is True:
                        msg = '{} {}'.format("E", err)
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                except Exception as e:
                    print("Cannot send error packet: ", e)

        #======================================== Screen Stuff =======================================================================
        try:

            cmd_handler()

            if button.value() ==1:
                if check_button(button) == 'boot':
                    shutdown()

            screen.send_PR(p_unit_noavg, p_unit)

            sendError = False
        except TypeError as t:
            pass
        except Exception as e:
            if sendError is False:
                sendError = True
                err = "Screen Send Error: " + str(e)
                print(err)
                screen.error(err)
                try:
                    if network is True:
                        msg = '{} {}'.format("E", err)
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                except Exception as e:
                    print("Cannot send error packet: ", e) 

        #==================================================== XBee Stuff ============================================================

        try:

            cmd_handler()
    
            if healthSend is True:
                try:
                    if network is True:
                        msg = '{} {} {} {} {} {} {}'.format("HR", health[0], health[1], health[2], health[3], health[4], health[5])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                        screen.change_page("manometer")
                        healthSend = False
                    else:
                        healthSend = True
                except Exception as e:
                    healthSend = True
                    err = "Health Report Not Sent: " + str(e)
                    print(err)
    
            cmd_handler()

            if saveSend is True:
                try:
                    if network is True:
                        msg = '{} {} {} {}'.format("S", save[0], save[1], save[2])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                        screen.change_page("manometer")
                        saveSend = False
                    else:
                        saveSend = True
                except Exception as e:
                    saveSend = True
                    err = "Save Data Not Sent: " + str(e)
                    print(err)
                    
            cmd_handler()

            if locationSend is True:
                try:
                    if network is True:
                        msg = '{} {} {} {} {} {} {}'.format("L", location[0], location[1], location[2], location[3], location[4], location[5])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                        screen.change_page("manometer")
                        locationSend = False
                    else:
                        locationSend = True
                except Exception as e:
                    locationSend = True
                    err = "Location Not Sent: " + str(e)
                    print(err)

            cmd_handler()
    
            if time.ticks_diff(time.ticks_ms(), last_health) > 180000 and bigNet == False:
                last_health = time.ticks_ms()
                try:
                    msg = '{} {} {} {} {}'.format("BH", round(voltage, 2), round(-1*current,2), round(percent,1), round(bTemp,1))
                    xbee.txData(COORDINATOR_ADDR, msg)
                    time.sleep(0.005)
                except Exception as e:
                    err = "Battery Health Send Error: " + str(e)
                    print(err)
                    screen.error(err)
    
                try:
                    msg = '{} {} {}'.format("PH", round(prs_sensor.get_temp(), 1),round(prs_sensor.zero_offset, 3))
                    xbee.txData(COORDINATOR_ADDR, msg)
                    time.sleep(0.005)
                except Exception as e:
                    err = "Pressure Health Send Error: " + str(e)
                    print(err)
                    screen.error(err)

            cmd_handler()
    
            if bigNet == False:
                if time.ticks_diff(time.ticks_ms(), last_send) > 1000:
                    last_send = time.ticks_ms()
                    try:
                        msg = '{} {}'.format("MP", round(p_inh2o,3))
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                    except Exception as e:
                        err = "Pressure Data Send Error: " + str(e)
                        print(err)
                        screen.error(err)
            else:
                if time.ticks_diff(time.ticks_ms(), last_send) > netTime:
                    last_send = time.ticks_ms()
                    try:
                        msg = '{} {}'.format("MP", round(p_inh2o,3))
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                    except Exception as e:
                        err = "Pressure Data Send Error: " + str(e)
                        print(err)
                        screen.error(err)
    
            cmd_handler()

            gc.collect()

            sendError = False
        except TypeError as t:
            pass
        except Exception as e:
            if sendError is False:
                sendError = True
                err = "Send Error: " + str(e)
                print(err)
                screen.error(err)
                try:
                    if network is True:
                        msg = '{} {}'.format("E", err)
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                except Exception as e:
                    print("Cannot send error packet: ", e)

        mainError = False
    except TypeError as t:
        pass
    except Exception as e:
        if mainError is False:
            mainError = True
            err = "Runtime Error: " + str(e)
            print(err)
            screen.error(err)
            try:
                if network is True:
                    msg = '{} {}'.format("E", err)
                    xbee.txData(COORDINATOR_ADDR, msg)
                    time.sleep(0.1)
            except Exception as e:
                print("Cannot send error packet: ", e)
