import time
import machine
import pyb
import stm
import json
import gc
import sys
from machine import Pin
from machine import I2C
from pyb import LED
from FrameGen import frame
from bmp585 import BMP585
from __init__ import PSNERGY_Manometer as Screen
import measurements as measurements
from MAX17263 import MAX17263


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
        return "boot"
    else:
        print("bad button press... sleeping")
        return "sleep"


def init_screen():
    global nextion, screen
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


def init_battery():
    global battery, screen, i2c_bus1
    try:
        battery = MAX17263(0x36, i2c_bus1, 0.01)
        screen.boot_progress("Initializing Battery...", 30)
        if battery.oc_voltage() <= 3.0:
            screen.change_page("deadBatt")
            time.sleep(10)
            en3v3.value(0)
            en5.value(0)
            time.sleep(3)
            stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8  # enable WKUP pin on PA0
            machine.deepsleep()
        time.sleep(0.01)
        screen.set_battery_percent(int(battery.batt_percent()))
    except Exception as e:
        print("Battery setup failure: ", e)
        screen.boot_progress("Battery Initialization Failure!", 30)


def init_pressures():
    global screen, o1, o2, o3, o4, sensor1, sensor2, sensor3, sensor4, sensor5, pressure1, pressure2, pressure3, pressure4, i2c_bus
    try:
        screen.boot_progress("Initializing Pressure Sensor...", 50)
        o1 = o2 = o3 = o4 = 0
        with open('ConfigValues.txt') as f:
            o1 = float(f.readline())
            o2 = float(f.readline())
            o3 = float(f.readline())
            o4 = float(f.readline())
        f.close()
        try:
            i2c_bus.writeto(112, b'\x10')
            sensor1 = BMP585(i2c_bus, channel=b'\x10', address=0x46, mux_address=0x70, offset=o1)
            print("successfully connected sensor 1")
            pressure1 = measurements.Measurement()
            pressure1.create_stream(50, 4)
        except Exception as e:
            print("Cannot connect sensor 1: ", e)
        try:
            i2c_bus.writeto(112, b'\x01')
            sensor2 = BMP585(i2c_bus, channel=b'\x01', address=0x46, mux_address=0x70, offset=o2)
            print("successfully connected sensor 2")
            pressure2 = measurements.Measurement()
            pressure2.create_stream(50, 4)
        except Exception as e:
            print("Cannot connect sensor 2: ", e)
        try:
            i2c_bus.writeto(112, b'\x08')
            sensor3 = BMP585(i2c_bus, channel=b'\x02', address=0x46, mux_address=0x70, offset=o3)
            print("successfully connected sensor 3")
            pressure3 = measurements.Measurement()
            pressure3.create_stream(50, 4)
        except Exception as e:
            print("Cannot connect sensor 3: ", e)
        try:
            i2c_bus.writeto(112, b'\x02')
            sensor4 = BMP585(i2c_bus, channel=b'\x08', address=0x46, mux_address=0x70, offset=o4)
            print("successfully connected sensor 4")
            pressure4 = measurements.Measurement()
            pressure4.create_stream(50, 4)
        except Exception as e:
            print("Cannot connect sensor 4: ", e)
        try:
            i2c_bus.writeto(112, b'\x04')
            sensor5 = BMP585(i2c_bus, channel=b'\x04', address=0x46, mux_address=0x70, offset=0)
            print("successfully connected sensor 5")
        except Exception as e:
            print("Cannot connect sensor 5: ", e)
    except Exception as e:
        print("Pressure Initialization Failure: ", e)
        screen.boot_progress("Pressure Sensor Initialization Failure!", 50)


def init_xbee():
    global screen, xbee, xbeeID
    try:
        screen.boot_progress("Initializing XBee...", 80)
        xbee = frame(2, 115200)
        xbeeID = DEVICE_ID
    except Exception as e:
        print("Xbee Initialization Failure: ", e)


def zero_sensors():
    global sensor1, sensor2, sensor3, sensor4, sensor5
    try:
        i2c_bus.writeto(112, sensor5.channel)
        d = sensor5.pressure
        ref_pressure = sensor5.raw_prs
        i2c_bus.writeto(112, sensor1.channel)
        d = sensor1.pressure
        sensor1.offset = round(sensor1.raw_prs - ref_pressure, 2)
        i2c_bus.writeto(112, sensor2.channel)
        d = sensor2.pressure
        sensor2.offset = round(sensor2.raw_prs - ref_pressure, 2)
        i2c_bus.writeto(112, sensor3.channel)
        d = sensor3.pressure
        sensor3.offset = round(sensor3.raw_prs - ref_pressure, 2)
        i2c_bus.writeto(112, sensor4.channel)
        d = sensor4.pressure
        sensor4.offset = round(sensor4.raw_prs - ref_pressure, 2)
        writeCalVals()
    except Exception as e:
        print(e)


def writeCalVals():
    global msg, xbee, network, screen, sensor1, sensor2, sensor3, sensor4, sensor5
    try:
        f = open('ConfigValues.txt', 'w')
        f.write(str(sensor1.offset) + '\n')
        f.write(str(sensor2.offset) + '\n')
        f.write(str(sensor3.offset) + '\n')
        f.write(str(sensor4.offset))
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


def cmd_handler():
    global screen, pressure, prs_sensor, prsUnits, location, network, msg, bmp_sensors, sensor1, sensor2, sensor3, sensor4, sensor5, locations, isTP, serviceMode
    global p_inh2o, xbee, first, newCal, temporary, locationSend, bigNet, netTime, pressure1, pressure2, pressure3, pressure4, pressure5, locationSend2
    instr = None
    xb = None
    try:
        instr = screen.read_instr()
        if instr != "" and instr is not None and instr != '' and instr != "b''":
            if instr[0] == "moving":
                if str(instr[1]) == "RT":
                    pressure1.stream_size = pressure2.stream_size = pressure3.stream_size = pressure4.stream_size = 1
                elif str(instr[1]) == "S":
                    pressure1.stream_size = pressure2.stream_size = pressure3.stream_size = pressure4.stream_size = 50
                elif str(instr[1]) == "M":
                    pressure1.stream_size = pressure2.stream_size = pressure3.stream_size = pressure4.stream_size = 100
                elif str(instr[1]) == "L":
                    pressure1.stream_size = pressure2.stream_size = pressure3.stream_size = pressure4.stream_size = 200
            elif instr[0] == "unit":
                sensor1.change_unit(prsUnits[int(instr[1])])
                sensor2.change_unit(prsUnits[int(instr[1])])
                sensor3.change_unit(prsUnits[int(instr[1])])
                sensor4.change_unit(prsUnits[int(instr[1])])
                sensor5.change_unit(prsUnits[int(instr[1])])
            elif instr[0] == "location":
                locations["level"] = instr[1]
                locations["side"] = instr[2]
                locations["zone"] = instr[3]
                locations["tube"] = instr[4]
                locations["media"] = instr[5]
                locations["type"] = instr[6]
                locationSend = True
            elif instr[0] == "location2":
                locations["level"] = instr[1]
                locations["side"] = instr[2]
                locations["zone"] = instr[3]
                locations["tube"] = instr[4]
                locations["media"] = instr[5]
                locations["type"] = instr[6]
                if locations["level"] == "NA" and locations["side"] == "NA" and locations["zone"] == "NA" and locations["tube"] == "NA" and locations["media"] == "NA" and locations["type"] == "NA":
                    isTP = False
                else:
                    isTP = True
                locationSend2 = True
            elif instr[0] == "zero":
                zero_sensors()
                newCal = True
            elif instr[0] == "dnd":
                screen.notifications(bool(instr[1]))
            elif instr[0] == "calibration_mode":
                serviceMode = True
                xbee.writeID(b'\x99')
            elif instr[0] == "service_mode":
                serviceMode = False
                xbee.writeID(b'\xAB\xCD')
            else:
                err = ("command not recognized: " + str(instr))
                print(err)
        if time.ticks_diff(time.ticks_ms(), first) > 5000:
            xb = xbee.xbRead()
            if xb != -1:
                if b'cmd' not in xb:
                    dic = xbee.receiveFrame(xb)
                    if dic != -1 and str(dic["Status"]) == "0x00" or str(dic["Status"]) == "0x0":
                        network = True
                        screen.set_signal_strength(2)
                    elif dic != -1 and str(dic["Status"]) != "0x00" and str(dic["Status"]) != "0x0":
                        network = False
                        screen.set_signal_strength(0)
                elif b'cmd' in xb and network is True:
                    xb = xb[15:-1]
                    xb = str(xb)
                    xb = xb[2:-1]
                    xb = xb.split()
                    if xb[0] == "cmd" and len(xb) > 1:
                        if xb[1] == "shutdown":
                            shutdown()
                        elif xb[1] == "sleep":
                            screen.write_global("sleep", xb[2])
                        elif xb[1] == "zero":
                            zero_sensors()
                            newCal = True
                        elif xb[1] == "unit":
                            if int(xb[2]) >= 0 and int(xb[2]) <= 3:
                                sensor1.change_unit(prsUnits[int(xb[2])])
                                sensor2.change_unit(prsUnits[int(xb[2])])
                                sensor3.change_unit(prsUnits[int(xb[2])])
                                sensor4.change_unit(prsUnits[int(xb[2])])
                                sensor5.change_unit(prsUnits[int(xb[2])])
                                screen.touch_component("{}{}".format("r", int(xb[2])+4))
                                screen.release_component("{}{}".format("r", int(xb[2])+4))
                        elif xb[1] == "moving":
                            pressure1.stream_size = pressure2.stream_size = pressure3.stream_size = pressure4.stream_size = int(xb[2])
                        elif xb[1] == "magic":
                            screen.magic_number(str(xb[2]))
                        elif xb[1] == "location":
                            screen.change_page("locale")
                            time.sleep(0.25)
                            screen.touch_component("bSubmit")
                            time.sleep(0.25)
                            screen.release_component("bSubmit")
                            time.sleep(0.5)
                            if isTP is True:
                                screen.change_page("locale2")
                                time.sleep(0.25)
                                screen.touch_component("bSubmit")
                                time.sleep(0.25)
                                screen.release_component("bSubmit")
                                time.sleep(0.25)
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
                        elif xb[1] == "offsets":
                            msg = '{} {} {} {} {}'.format("OS", sensor1.offset, sensor2.offset, sensor3.offset, sensor4.offset)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.5)
                        else:
                            err = ("xb command not recognized: " + str(xb))
                            print(err)
                        if serviceMode is True:
                            msg = '{} {}'.format("LG", xb[-1])
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.5)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.5)
                            xbee.txData(COORDINATOR_ADDR, msg)
            del xb
        del instr
    except TypeError:
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
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8  # enable WKUP pin on PA0
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


def reset_3v_bus():
    global i2c_bus, i2c_bus1, en3v3, screen
    try:
        en3v3.value(0)
        scl = Pin('B10', Pin.OUT, value=1)
        sda = Pin('B11', Pin.IN)
        for _ in range(9):
            scl.low()
            time.sleep_us(5)
            scl.high()
            time.sleep_us(5)
        scl.high()
        time.sleep_us(5)
        sda = Pin('B11', Pin.OUT)
        sda.value(1)
        time.sleep_us(5)
        i2c_bus = I2C(2)
        i2c_bus1 = I2C(1)
        time.sleep(0.01)
        en3v3.value(1)
        time.sleep(0.01)
        init_battery()
        init_pressures()
        init_xbee()
        screen.error("Reset 3V bus and Sensor Bus.")
    except Exception as e:
        err = "3V bus reset error: " + str(e)
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
    pyb.freq(168000000)
    AWAKE_START = time.ticks_ms()
    button = Pin('A1', Pin.IN, pull=Pin.PULL_DOWN)
    en5 = Pin('C9', Pin.OUT_PP)
    en5.value(0)
    en3v3 = Pin('A15', Pin.OUT_PP)
    en3v3.value(0)
    tag = Pin('B1', Pin.IN, pull=Pin.PULL_DOWN)
    xbee = frame(2, 115200)
    while True:
        if button.value() == 1:
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
                stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8  # enable WKUP pin on PA0
                machine.deepsleep()
    time.sleep(0.1)
    en5.value(1)
    en3v3.value(1)
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
    time.sleep(1)
    init_screen()
    time.sleep(1)
    init_battery()
    init_pressures()
    init_xbee()
    p_inh2o_noavg1 = p_inh2o_noavg2 = p_inh2o_noavg3 = p_inh2o_noavg4 = p_inh2o_noavg5 = p_inh2o1 = p_inh2o2 = p_inh2o3 = p_inh2o4 = p_inh2o5 = last_health = 0
    last_normal = last_send = p1diff = p2diff = current = percent = voltage = p1h = p2h = p3h = p4h = 0
    locationSend = newCal = network = bigNet = sscError = batError = senseError = screenError = sendError = mainError = locationSend2 = isTP = serviceMode = False
    netTime = 2000
    prsUnits = ["psi", "mbar", "inh2o", "kpa"]
    values = "{}"
    locations = "{}"
    health = "{}"
    values = json.loads(values)
    locations = json.loads(locations)
    health = json.loads(health)
    msg = ""
    LED(4).on()
    screen.boot_progress("Boot Successful!", 100)
    time.sleep(0.1)
    screen.change_page("manometer")
    xbee.writeID(b'\xAB\xCD')
    beginStartLoop = time.ticks_ms()
    first = time.ticks_ms()
except Exception as e:
    err = "Init Variables Failure: " + str(e)
    print(err)
    screen.error(err)

while True:
    try:
        # ===================================================== Sense Stuff =================================================================
        try:
            cmd_handler()
            try:
                i2c_bus.writeto(112, sensor1.channel)
                p = round(sensor1.pressure, 2)
                p1h = sensor1.inh2o
                t1 = round(sensor1.temperature, 1)
                pressure1.add_point(p)
                p_inh2o1 = round(pressure1.average(), 2)
                p_inh2o_noavg1 = round(p, 2)

                i2c_bus.writeto(112, sensor2.channel)
                p = round(sensor2.pressure, 2)
                p2h = sensor2.inh2o
                t2 = round(sensor2.temperature, 1)
                pressure2.add_point(p)
                p_inh2o2 = round(pressure2.average(), 2)
                p_inh2o_noavg2 = round(p, 2)

                i2c_bus.writeto(112, sensor3.channel)
                p = round(sensor3.pressure, 2)
                p3h = sensor3.inh2o
                t3 = round(sensor3.temperature, 1)
                pressure3.add_point(p)
                p_inh2o3 = round(pressure3.average(), 2)
                p_inh2o_noavg3 = round(p, 2)

                i2c_bus.writeto(112, sensor4.channel)
                p = round(sensor4.pressure, 2)
                p4h = sensor4.inh2o
                t4 = round(sensor4.temperature, 1)
                pressure4.add_point(p)
                p_inh2o4 = round(pressure4.average(), 2)
                p_inh2o_noavg4 = round(p, 2)

                i2c_bus.writeto(112, sensor5.channel)
                d = round(sensor5.pressure, 2)
                p5h = round(sensor5.raw_prs, 2)

                p1diff = p_inh2o1 - p_inh2o2
                p1diff = 0.0 if -0.05 < p1diff < 0.05 else p1diff
                p2diff = p_inh2o3 - p_inh2o4
                p2diff = 0.0 if -0.05 < p2diff < 0.05 else p2diff
                p1diff_na = p_inh2o_noavg1 - p_inh2o_noavg2
                p1diff_na = 0.0 if -0.05 < p1diff_na < 0.05 else p1diff_na
                p2diff_na = p_inh2o_noavg3 - p_inh2o_noavg4
                p2diff_na = 0.0 if -0.05 < p2diff_na < 0.05 else p2diff_na
                p1diff_in = p1h - p2h
                p1diff_in = 0.0 if -0.05 < p1diff_in < 0.05 else p1diff_in
                p2diff_in = p3h - p4h
                p2diff_in = 0.0 if -0.05 < p2diff_in < 0.05 else p2diff_in

                values["p1"] = round(p1diff_in, 2)
                values["p2"] = round(p2diff_in, 2)
                health["t1"] = round(t1, 2)
                health["t2"] = round(t2, 2)
                health["t3"] = round(t3, 2)
                health["t4"] = round(t4, 2)
                sscError = False
            except TypeError:
                pass
            except Exception as e:
                if sscError is False:
                    sscError = True
                    err = "Pressure Sensor Error: " + str(e)
                    print(err)
                    screen.error(err)
                    try:
                        if network is True:
                            msg = '{} {}'.format("E", err)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.1)
                    except Exception as e:
                        print("Cannot send error packet: ", e)
                    reset_3v_bus()
            cmd_handler()
            try:
                voltage = battery.oc_voltage()
                current = battery.current_draw()
                percent = battery.batt_percent()
                bTemp = battery.batt_temp()
                health["battery_voltage"] = round(voltage, 2)
                health["battery_current_draw"] = round(current, 2)
                health["battery_percentage"] = round(percent, 2)
                health["battery_temp"] = round(bTemp, 2)
                if voltage <= 3.0:
                    shutdown()
                batError = False
            except TypeError:
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
        except TypeError:
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
        # ====================================================== Screen Stuff =======================================================================
        try:
            cmd_handler()
            if button.value() == 1:
                if check_button(button) == 'boot':
                    shutdown()
            screen.set_battery_percent(int(percent))
            screen.send_PR(p2diff, p1diff)
            sendError = False
        except TypeError:
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
        # ==================================================== XBee Stuff ============================================================
        try:
            cmd_handler()
            if locationSend is True:
                try:
                    if network is True:
                        msg = '{} {} {} {} {} {} {}'.format("L", locations["level"], locations["side"], locations["zone"], locations["tube"], locations["media"], locations["type"])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                        screen.change_page("manometer")
                        locationSend = False
                    else:
                        locationSend = True
                except Exception as e:
                    locationSend = True
                    err = "Location Not Sent: " + str(e)
                    print(err)
            if locationSend2 is True:
                try:
                    if network is True:
                        msg = '{} {} {} {} {} {} {}'.format("SL", locations["level"], locations["side"], locations["zone"], locations["tube"], locations["media"], locations["type"])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                        screen.change_page("manometer")
                        locationSend2 = False
                    else:
                        locationSend2 = True
                except Exception as e:
                    locationSend2 = True
                    err = "Location 2 Not Sent: " + str(e)
                    print(err)
            cmd_handler()
            if time.ticks_diff(time.ticks_ms(), last_health) > 180000 and bigNet is False:
                last_health = time.ticks_ms()
                try:
                    if network is True:
                        msg = '{} {} {} {} {} {} {} {} {}'.format("DH", health["battery_voltage"], health["battery_current_draw"], health["battery_percentage"], health["battery_temp"], health["t1"],
                                                                  health["t2"], health["t3"], health["t4"])
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                except Exception as e:
                    err = "Device Health Send Error: " + str(e)
                    print(err)
                    screen.error(err)
            cmd_handler()
            if bigNet is False:
                if time.ticks_diff(time.ticks_ms(), last_send) > 1000:
                    last_send = time.ticks_ms()
                    try:
                        if isTP is False:
                            msg = '{} {} {}'.format("OP", values["p2"], p5h)
                        else:
                            msg = '{} {} {} {}'.format("TP", values["p2"], values["p1"], p5h)
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
                        if isTP is False:
                            msg = '{} {} {}'.format("OP", values["p2"], p5h)
                        else:
                            msg = '{} {} {} {}'.format("TP", values["p2"], values["p1"], p5h)
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.005)
                    except Exception as e:
                        err = "Pressure Data Send Error: " + str(e)
                        print(err)
                        screen.error(err)
            cmd_handler()
            gc.collect()
            sendError = False
        except TypeError:
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
    except TypeError:
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
