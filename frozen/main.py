import time, machine, stm, pyb, json, gc
from machine import Pin, I2C, UART
from pyb import Pin, LED
from FrameGen import frame

pyb.freq(168000000)
# BOOT UP
sys_wake = Pin('A8', Pin.OUT_PP)
sys_wake.value(0)
AWAKE_START = time.ticks_ms()
button = Pin('A1', Pin.IN, pull=Pin.PULL_DOWN)
tmc_diag = Pin('C2', Pin.IN, pull=Pin.PULL_DOWN)
tag = Pin('B1', Pin.IN, pull=Pin.PULL_DOWN)
xbeeEn = Pin('B8', Pin.OUT_PP, pull=Pin.PULL_UP)
xbeeEn.value(0)
xbee = frame(2, 115200)
adc = pyb.ADC('C5')
gp = Pin('C0', Pin.OUT_PP, pull=Pin.PULL_DOWN)
gp.value(0)
en5 = Pin('C9', Pin.OUT_PP)
en5.value(0)
en3v3 = Pin('A15', Pin.OUT_PP)
en3v3.value(0)

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
            en3v3.on()
            en5.on()
            break
    elif tag.value() == 1:
        en3v3.on()
        en5.on()
        xbeeEn.value(1)
        machine.UART(2).deinit()
        machine.UART(6).deinit()
        continue
    else:
        if time.ticks_diff(time.ticks_ms(), AWAKE_START) < 10000:
            pass
        else:
            stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
            machine.deepsleep()

en5.value(1)
en3v3.value(1)
xbeeEn.value(1)
sys_wake.value(1)

# INITIAL CONFIG OF ANALYZER
import aducm355 as Electrochem, math, measurements, TMC7300 as tmc, O2_Luminox, PRS_TruStabilitySSC
from __init__ import PSNERGY_Screen as Screen
from BME280 import BME280
from MAX17263 import MAX17263


pumpSTDBY = pyb.Pin(pyb.Pin.board.C1, Pin.OUT_PP, pull=Pin.PULL_DOWN)
pumpEN = pyb.Pin(pyb.Pin.board.C3, Pin.OUT_PP, pull=Pin.PULL_DOWN)
pump_uart = machine.UART(3,115200)
pump = tmc.TMC_UART(pump_uart, 0x00)

jfile = open("AnalyzerConfig.json")
Configuration = json.load(jfile)
jfile.close()
DEVICE_ID = Configuration["XbeeConfig"]["deviceID"]
i2c_bus = I2C(1)
i2c_bus1 = I2C(2)
COORDINATOR_ADDR = "0000000000000000"

#screen setup
try:
    nextion = machine.UART(6, 115200)
    nextion.init(baudrate=115200, bits=8, parity=None, stop=1, timeout_char=1, timeout=3)
    screen = Screen(nextion, debug_mode=False)
    time.sleep(0.1)
    screen.set_device_name(str(DEVICE_ID))
    screen.write_global_text("boot", "tcodeVersion", "Combustion Analyzer v3.3.10")
    screen.write_global_text("boot", "tscreenVersion", "Screen Version v3.0.5")
    screen.boot_progress("Initializing Screen...", 20)
    screen.set_signal_strength(1)
except Exception as e:
    print("Screen Initialization error: ", e)

#battery setup
try:
    battery = MAX17263(0x36, i2c_bus, 0.01)
    screen.boot_progress("Initializing Battery...", 30)
    if battery.oc_voltage() <= 3.2:
        screen.change_page("deadBatt")
        time.sleep(10)
        en3v3.value(0)
        en5.value(0)
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
        machine.deepsleep()
    time.sleep(0.01)
except Exception as e:
    print("Battery setup failure: ", e)
try:
    screen.set_battery_percent(int(battery.batt_percent()))
    time.sleep(0.01)
    screen.write_global("boot.percent.aph", "127")
    screen.write_global("boot.batt.aph", "127")
except Exception as e:
    print("Cannot set battery percent on screen: ", e)
#t/h setup
try:
    screen.boot_progress("Initializing Temperature Sensor...", 40)
    bme = BME280(mode=1, address=0x76, i2c=machine.I2C(2))
except Exception as e:
    print("Temp Initialization Failure: ", e)
#pressure setup
try:
    screen.boot_progress("Initializing Pressure Sensor...", 50)
    while True:
        time.sleep(0.05)
        prs_sensor = PRS_TruStabilitySSC.SSC(0x28, 2, -138.5379, 138.5379)
        break
    pressure = measurements.Measurement()
    pressure.create_stream(20, 4)
except Exception as e:
    try:
        while True:
            time.sleep(0.05)
            prs_sensor = PRS_TruStabilitySSC.SSC(0x38, 2, -138.5379, 138.5379)
            break
        pressure = measurements.Measurement()
        pressure.create_stream(20, 4)
    except Exception as e:
        print("Pressure Initialization Failure: ", e)

time.sleep(0.01)

# Pump setup
try:
    screen.write_global_text("boot", "tcodeVersion", "Combustion Analyzer v3.3.10")
    screen.write_global_text("boot", "tscreenVersion", "Screen Version v3.0.5")
    screen.boot_progress("Initializing Pumps...", 60)
    pumpSTDBY.value(0)
    pumpEN.value(0)
    
    pumpSTDBY.value(1)
    time.sleep(0.01)
    
    pump.write_reg(tmc.GCONF, [0x00,0x00,0x00,0x01])
    pump.write_reg(tmc.CURRENT_LIMIT, [0x00,0x00,0x1f,0x01])
    pump.setPercent('b', 0, '+')
    pump.setPercent('a', 0, '+')
    pumpEN.value(1)
except Exception as e:
    print("Pump Initialization Failure: ", e)

sys_wake.value(0)
time.sleep(0.1)
sys_wake.value(1)

@micropython.native
def loadCalibration():  #load values from config text into local variables
    global ec_co_sensor, ec_no_sensor, o2offset, o2gain, prs_sensor, coGain, noxGain
    try:
        config = list(range(13))
        iterator = 0
        chars = 0
        name = ""
        with open('ConfigValues.txt') as f: 
            for iterator in range(len(config)): #Loads the rest of the number values into config array
                config[iterator] = f.readline()
                iterator+=1
        f.close()
        ec_co_sensor.ref_Low = int(config[0])
        ec_co_sensor.ref_High = int(config[1])
        ec_co_sensor.raw_Low = int(config[2])
        ec_co_sensor.raw_High = int(config[3])
        ec_no_sensor.ref_Low = int(config[4])
        ec_no_sensor.ref_High = int(config[5])
        ec_no_sensor.raw_Low = int(config[6])
        ec_no_sensor.raw_High = int(config[7])
        o2offset = float(config[8])
        o2gain = float(config[9])
        prs_sensor.zero_offset = float(config[10])
        coGain = float(config[11])
        noxGain = float(config[12])
        return "Done loading"
    except Exception as e:
        err = "EC Calibration Load Error: " + str(e)
        print(err)
        screen.error(err)


while True:
    try:
        sys_wake.value(1)
        print("Waking EC...")
        screen.boot_progress("Waking Electrochemical Sensors...", 65)
        time.sleep(1)
        print(i2c_bus.scan())
        machine.I2C(1).writeto(77, b'\x02')
        machine.I2C(1).writeto(77, b'\x02')
        machine.I2C(1).writeto(76, b'\x02')
        machine.I2C(1).writeto(76, b'\x02') 
        print(i2c_bus.scan()) 
        break
    except Exception as e:
        continue

sys_wake.value(1)

##analyzer setup
try:
    screen.write_global_text("boot", "tcodeVersion", "Combustion Analyzer v3.3.10")
    screen.write_global_text("boot", "tscreenVersion", "Screen Version v3.0.5")
    screen.set_signal_strength(1)
    screen.boot_progress("Initializing Oxygen Sensor...", 70)
    screen.set_device_name(str(DEVICE_ID))
    o2_uart = machine.UART(5, 9600)
    o2_sensor = O2_Luminox.Luminox(o2_uart, o2_offset=float(Configuration["Calibration"]["luminox_offset"]))
    oxy = measurements.Measurement()
    oxy.create_stream(2, 1)
    time.sleep(.01)
except Exception as e:
    print("O2 Initialization Failure: ", e)
screen.boot_progress("Initializing Electrochemical Sensors...", 80)

try:
    ec_co_sensor = Electrochem.CO(addr=76, bus=i2c_bus)
    ec_co_sensor.configure_sensor(vbias=Configuration["Electrochem"]["CO"]["vbias"],vzero_voltage=Configuration["Electrochem"]["CO"]["vzero_voltage"],sensor_sens=Configuration["Electrochem"]["CO"]["sensor_sens"],tia_gain=Configuration["Electrochem"]["CO"]["tia_gain"],rload=Configuration["Electrochem"]["CO"]["rload"])
    ec_co_sensor.setCalValues(raw_Low=int(Configuration["Electrochem"]["CO"]["raw_Low"]),raw_High=int(Configuration["Electrochem"]["CO"]["raw_High"]),ref_Low=int(Configuration["Electrochem"]["CO"]["ref_Low"]),ref_High=int(Configuration["Electrochem"]["CO"]["ref_High"]))
except Exception as e:
    print("CO Initialization Failure: ", e)
try:
    ec_no_sensor = Electrochem.NO(addr=77, bus=i2c_bus)
    ec_no_sensor.configure_sensor(vbias=Configuration["Electrochem"]["NO"]["vbias"],vzero_voltage=Configuration["Electrochem"]["NO"]["vzero_voltage"],sensor_sens=Configuration["Electrochem"]["NO"]["sensor_sens"],tia_gain=Configuration["Electrochem"]["NO"]["tia_gain"],rload=Configuration["Electrochem"]["NO"]["rload"])
    ec_no_sensor.setCalValues(raw_Low=int(Configuration["Electrochem"]["NO"]["raw_Low"]),raw_High=int(Configuration["Electrochem"]["NO"]["raw_High"]),ref_Low=int(Configuration["Electrochem"]["NO"]["ref_Low"]),ref_High=int(Configuration["Electrochem"]["NO"]["ref_High"]))
except Exception as e:
    print("NO Initialization Failure: ", e)

#xbee setup
try:
    screen.boot_progress("Initializing XBee...", 90)
    xbee = frame(2, 115200)
    xbeeID = DEVICE_ID
except Exception as e:
    print("Xbee Initialization Failure: ", e)

time.sleep(0.01)

@micropython.native
def writeCalVals():
    global ec_co_sensor, ec_no_sensor, screen, prs_sensor, o2offset, o2gain, prs_sensor, coGain, noxGain, xbee, msg
    try:
        f = open('ConfigValues.txt', 'w')
        f.write(str(ec_co_sensor.ref_Low)+'\n')
        f.write(str(ec_co_sensor.ref_High)+'\n')
        f.write(str(ec_co_sensor.raw_Low)+'\n')
        f.write(str(ec_co_sensor.raw_High)+'\n')
        f.write(str(ec_no_sensor.ref_Low)+'\n')
        f.write(str(ec_no_sensor.ref_High)+'\n')
        f.write(str(ec_no_sensor.raw_Low)+'\n')
        f.write(str(ec_no_sensor.raw_High)+'\n')
        f.write(str(o2offset)+'\n')
        f.write(str(o2gain)+'\n')
        f.write(str(prs_sensor.zero_offset)+'\n')
        f.write(str(coGain)+'\n')
        f.write(str(noxGain))
        f.close()
    except Exception as e:
        err = "Write Calibration Values to File Error: " + str(e)
        print(err)
        screen.error(err)
        try:
            msg = '{} {}'.format("E", err)
            xbee.txData(COORDINATOR_ADDR, msg)
            time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

def reset_tmc():
    global pumpSTDBY, pumpEN, pump, xbee, msg, tmc
    try:
        pumpSTDBY.value(0)
        pumpEN.value(0)
        pumpSTDBY.value(1)
        time.sleep(0.01)
        pump.write_reg(tmc.GCONF, [0x00,0x00,0x00,0x01])
        pump.write_reg(tmc.CURRENT_LIMIT, [0x00,0x00,0x1f,0x01])
        pump.setPercent('b', 100, '+')
        pump.setPercent('a', 0, '+')
        pumpEN.value(1)
    except Exception as e:
        print("Cannot reset tmc: ", e)
        try:
            msg = '{} {}'.format("E", err)
            xbee.txData(COORDINATOR_ADDR, msg)
            time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

def cmd_handler():
    global screen, pump, pump_saved, inSafe, pressure, prs_sensor, prsUnits, location, save, oxy, co, healthSend, p_inh2o, newo2, tempOffset, tempGain, safeToggle, override, coGain, msg, xbee
    global ec_co_sensor, ec_no_sensor, coraw, noraw, status, inStop, cal, cancel, o2gain, o2offset, saveSend, health, first, network, newCal, temporary, lastHum2, humid, locationSend, bigNet, netTime
    instr = ""
    try:
        instr = screen.read_instr()
        if instr != "" and instr != None and instr != '':
            print(instr)
            if instr[0] == "pump":
                pump_saved = int(instr[1])
                pump.setPercent('b', 100, '+')
            elif instr[0] == "override":
                override = int(instr[1])
                if override == 1:
                    inSafe = False
                    safeToggle = False
                    inProtect = False
            elif instr[0] == "dnd":
                screen.notifications(int(instr[1]))
            elif instr[0] == "state":
                if instr[1] == "safe" and override == 0:
                    inSafe = True
                    safeToggle = True
                    if humid is True:
                        lastHum2 = time.ticks_ms()
                        humid = False
                elif instr[1] == "run":
                    inStop = False
                    safeToggle = False
                    if humid is True:
                        lastHum2 = time.ticks_ms()
                        humid = False
                elif instr[1] == "stop":
                    inStop = True
                    safeToggle = False
                elif instr[1] == "pumpCal":
                    safeToggle = True
                    inSafe = True
            elif instr[0] == "moving":
                pressure.stream_size = int(instr[1])
                temporary = pressure.stream_size
                screen.notify("Prs Average Changed!", "{}{}".format("New Average: ", int(instr[1])))
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "unit":
                prs_sensor.change_unit(prsUnits[int(instr[1])])
                screen.notify("Prs Units Changed!", "{}{}".format("New Units: ", prsUnits[int(instr[1])]))
                screen.write_global("tmBar.en", 1)
                if int(instr[1]) == 2:
                    screen.change_text_color("home", "tPRu", "BLACK")
                    screen.change_text_color("manometer", "tRTu", "BLACK")
                    screen.change_text_color("manometer", "tMAu", "BLACK")
                else:
                    screen.change_text_color("home", "tPRu", "RED")
                    screen.change_text_color("manometer", "tRTu", "RED")
                    screen.change_text_color("manometer", "tMAu", "RED")
            elif instr[0] == "calvals":
                screen.populate_calvals(round(o2gain, 2), round(o2offset, 2), ec_co_sensor.raw_High, ec_co_sensor.raw_Low, ec_no_sensor.raw_High, ec_no_sensor.raw_Low, round(prs_sensor.zero_offset, 2))
            elif instr[0] == "location":
                location = instr[1:]
                locationSend = True
            elif instr[0] == "save":    # "save flue initial", "save pressure final"
                if instr[1] == "pressure":
                    save[0] = "pressure"
                    save[1] = str(round(pressure.average(),1))
                    save[2] = str(instr[2])
                    save[3] = ""
                elif instr[1] == "flue":
                    save[0] = "flue"
                    save[1] = str(round(oxy.average(),1))
                    save[2] = str(int(co))
                    save[3] = str(instr[2])
                saveSend = True
            elif instr[0] == "report":
                health = instr[1:]
                healthSend = True
            elif instr[0] == "zero":
                screen.notify("Zeroing Pressure...", "Wait for finish notification!")
                screen.write_global("tmBar.en", 1)
                loadCalibration()
                prs_sensor.zero()
                writeCalVals()
                newCal = True
                screen.notify("Pressure Offset Saved!", "")
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "cal":
                cal = str(instr[1])
            elif instr[0] == "o2offset":
                loadCalibration()
                o2offset = (o2_sensor.ppo2/o2_sensor.pressure)*100
                time.sleep(0.01)
                screen.notify("O2 Offset Saved!", "")
                screen.write_global("tmBar.en", 1)
                writeCalVals()
                newCal = True
            elif instr[0] == "o2gain":
                loadCalibration()
                o2gain = 20.7/(((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)
                time.sleep(0.01)
                screen.notify("O2 Gain Saved!", "")
                screen.write_global("tmBar.en", 1)
                writeCalVals()
                newCal = True
            elif instr[0] == "coLow":
                loadCalibration()
                ec_co_sensor.ref_Low = int(instr[1])
                ec_co_sensor.raw_Low = int(instr[2])
                writeCalVals()
                newCal = True
                screen.notify("CO Low Saved!", "")
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "coHigh":
                loadCalibration()
                ec_co_sensor.ref_High = int(instr[1])
                ec_co_sensor.raw_High = int(instr[2])
                writeCalVals()
                newCal = True
                screen.notify("CO High Saved!", "")
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "noLow":
                loadCalibration()
                ec_no_sensor.ref_Low = int(instr[1])
                ec_no_sensor.raw_Low = int(instr[2])
                writeCalVals()
                newCal = True
                screen.notify("NO Low Saved!", "")
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "noHigh":
                loadCalibration()
                ec_no_sensor.ref_High = int(instr[1])
                ec_no_sensor.raw_High = int(instr[2])
                writeCalVals()
                newCal = True
                screen.notify("NO High Saved!", "")
                screen.write_global("tmBar.en", 1)
            elif instr[0] == "cancel":
                cancel = True
            elif instr[0] == "dnd":
                screen.notifications(bool(instr[1]))
            elif instr[0] == "add":
                loadCalibration()
                o2offset += 0.1
                writeCalVals()
                newCal = True
            elif instr[0] == "sub":
                loadCalibration()
                o2offset -= 0.1
                writeCalVals()
                newCal = True
            elif instr[0] == "pumpGain":
                loadCalibration()
                CO_TARGET = int(instr[1])
                CO_ACTUAL = int(instr[2])
                z = CO_TARGET/CO_ACTUAL
                coGain = round(z, 3)
                screen.notify("Pump Cal Submitted!", "New Gain: " + str(round(coGain,3)))
                screen.write_global("tmBar.en", 1)
                writeCalVals()
                newCal = True
            elif instr[0] == "bignet":
                if int(instr[1]) > 1 and int(instr[1]) <= 60:
                    netTime = int(instr[1])*1000
                    bigNet = True
                else:
                    netTime = 1000
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
                            if xb[2] == "pressure":
                                save[0] = "pressure"
                                save[1] = str(round(pressure.average(),1))
                                save[2] = str(xb[3])
                                save[3] = ""
                            elif xb[2] == "flue":
                                save[0] = "flue"
                                save[1] = str(round(oxy.average(),1))
                                save[2] = str(int(co))
                                save[3] = str(xb[3])
                            saveSend = True
                        elif xb[1] == "mode":
                            if xb[2] == "stop":
                                screen.stop_mode()
                                inStop = True
                                inSafe = False
                                safeToggle = False
                                status = 4
                                pump.setPercent('b', 0, '+')
                                pump.setPercent('a', 0, '+')
                            elif xb[2] == "run":
                                screen.run_mode()
                                inSafe = False
                                safeToggle = False
                                inStop = False
                                status = 0
                                pump.setPercent('b', 100, '+')
                                pump.setPercent('a', 0, '+')
                            elif xb[2] == "safe" and override == 0:
                                screen.safe_mode()
                                inSafe = True
                                inStop = False
                                status = 1
                                safeToggle = True
                                if humid is True:
                                    lastHum2 = time.ticks_ms()
                                    humid = False
                                pump.setPercent('b', 100, '+')
                                pump.setPercent('a', 100, '+')
                        elif xb[1] == "magic":
                            screen.magic_number(str(xb[2]))
                            screen.notify("Magic Number Updated!", "{}{}".format("New Magic Number: ", str(xb[2])))
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "location":
                            locationSend = True
                        elif xb[1] == "bigneton":
                            if int(xb[2]) > 1 and int(xb[2]) <= 60:
                                netTime = int(xb[2])*1000
                                bigNet = True
                            else:
                                netTime = 1000
                        elif xb[1] == "bignetoff":
                            bigNet = False
                            netTime = 1000
                        elif xb[1] == "battery_saver_on":
                            time.sleep(0.25)
                            screen.change_page("general")
                            time.sleep(0.25)
                            screen.touch_component("bsON")
                            time.sleep(0.25)
                            screen.release_component("bsON")
                        elif xb[1] == "battery_saver_off":
                            screen.write_global("sleep", 0)
                            time.sleep(0.25)
                            screen.change_page("general")
                            time.sleep(0.25)
                            screen.touch_component("bsOFF")
                            time.sleep(0.25)
                            screen.release_component("bsOFF")
                        elif xb[1] == "home":
                            time.sleep(0.1)
                            screen.change_page("home")
                        elif xb[1] == "zero":
                            screen.notify("Zeroing Pressure...", "Wait for finish notification!")
                            screen.write_global("tmBar.en", 1)
                            loadCalibration()
                            prs_sensor.zero()
                            writeCalVals()
                            newCal = True
                            screen.notify("Pressure Offset Saved!", "")
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "o2offset":
                            reference = float(xb[2])
                            if reference >= 0 and reference <= 4:
                                o2offset = (o2_sensor.ppo2/o2_sensor.pressure)*100 - reference
                                loadCalibration()
                                time.sleep(0.01)
                                screen.notify("O2 Offset Saved!", "")
                                screen.write_global("tmBar.en", 1)
                                writeCalVals()
                                newCal = True
                        elif xb[1] == "o2gain":
                            loadCalibration()
                            o2gain = 20.7/(((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)
                            time.sleep(0.01)
                            screen.notify("O2 Gain Saved!", "")
                            screen.write_global("tmBar.en", 1)
                            writeCalVals()
                            newCal = True
                        elif xb[1] == "colow":
                            loadCalibration()
                            ec_co_sensor.ref_Low = int(xb[2])
                            ec_co_sensor.raw_Low = int(xb[3])
                            writeCalVals()
                            newCal = True
                            screen.notify("CO Low Saved!", "")
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "cohigh":
                            loadCalibration()
                            ec_co_sensor.ref_High = int(xb[2])
                            ec_co_sensor.raw_High = int(xb[3])
                            writeCalVals()
                            newCal = True
                            screen.notify("CO High Saved!", "")
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "nolow":
                            loadCalibration()
                            ec_no_sensor.ref_Low = int(xb[2])
                            ec_no_sensor.raw_Low = int(xb[3])
                            writeCalVals()
                            newCal = True
                            screen.notify("NO Low Saved!", "")
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "nohigh":
                            loadCalibration()
                            ec_no_sensor.ref_High = int(xb[2])
                            ec_no_sensor.raw_High = int(xb[3])
                            writeCalVals()
                            newCal = True
                            screen.notify("NO High Saved!", "")
                            screen.write_global("tmBar.en", 1)
                        elif xb[1] == "add":
                            loadCalibration()
                            o2offset += 0.1
                            writeCalVals()
                            newCal = True
                        elif xb[1] == "sub":
                            loadCalibration()
                            o2offset -= 0.1
                            writeCalVals()
                            newCal = True
                        elif xb[1] == "pumpgain":
                            loadCalibration()
                            CO_TARGET = int(xb[2])
                            CO_ACTUAL = int(xb[3])
                            z = CO_TARGET/CO_ACTUAL
                            coGain = round(z, 3)
                            screen.notify("Pump Cal Submitted!", "New Gain: " + str(round(coGain,3)))
                            screen.write_global("tmBar.en", 1)
                            writeCalVals()
                            newCal = True
                        else:
                            print("Command not recognized!")
            del xb
        del instr
    except TypeError as t:
        print(t)
        try:
            msg = '{} {}'.format("E", err)
            xbee.txData(COORDINATOR_ADDR, msg)
            time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)
    except Exception as e:
        err = "Instruction Reading Failure: " + instr
        print(err)
        try:
            msg = '{} {}'.format("E", err)
            xbee.txData(COORDINATOR_ADDR, msg)
            time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

@micropython.native
def shutdown():
    global pump, co, ec_co_sensor, ec_co, en3v3, en5, cancel, battLow, xbee, p_inh2o, msg, inStop, inSafe, inProtect
    try:
        beginPurge = time.ticks_ms()
        cancel = False
        if battLow is False and co > 20:
            pump.setPercent('a', 100, '+')
            pump.setPercent('b', 100, '+')
            screen.change_page("shutdown")
            while co > 20 and time.ticks_diff(time.ticks_ms(), beginPurge) < 30000: #30 seconds Purge max
                screen.send_shutdown("CO Too High!", "CO Target: <20ppm", co)
                if cancel is True:
                    screen.stop_mode()
                    inSafe = inProtect = cancel = False
                    inStop = True
                    pump.setPercent('a', 0, '+')
                    pump.setPercent('b', 0, '+')
                    screen.change_page("home")
                    screen.notify("Shutdown Canceled!")
                    screen.write_global("tmBar.en", 1)
                elif cancel is False:
                    co = ec_co_sensor.readGas()
                    co = round((((co - ec_co_sensor.raw_Low) * (ec_co_sensor.ref_High - ec_co_sensor.ref_Low)) / (ec_co_sensor.raw_High - ec_co_sensor.raw_Low)) + ec_co_sensor.ref_Low)
                cmd_handler()
        elif battLow is False:
            pump.setPercent('a', 100, '+')
            pump.setPercent('b', 100, '+')
            screen.change_page("shutdown")
            ctr = 100
            while ctr > 0:
                cmd_handler()
                if cancel is True:
                    screen.stop_mode()
                    inSafe = inProtect = cancel = False
                    inStop = True
                    pump.setPercent('a', 0, '+')
                    pump.setPercent('b', 0, '+')
                    time.sleep(0.1)
                    screen.change_page("home")
                    screen.notify("Shutdown Canceled!")
                    screen.write_global("tmBar.en", 1)
                time.sleep(0.05)
                screen.send_shutdown("Shutting Down", "Time Left: ", str(int(ctr/10)))
                time.sleep(0.05)
                ctr-=1
        status = 4
        time.sleep(0.1)
        sys_wake.value(0)
        xbee.deinit()
        en3v3.value(0)
        en5.value(0)
        time.sleep(0.05)
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
        machine.deepsleep()
    except Exception as e:
        print("Shutdown Error: ", e)
        try:
            msg = '{} {}'.format("E", err)
            xbee.txData(COORDINATOR_ADDR, msg)
            time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)

try:
    rawScaledCO = p_inh2o = p_unit = p_unit_noavg = current = percent = voltage = mcu_temp = ptemp = inline_pressure = co = no = status = override = 0
    co_calculated = co_calculated_raw = no_calculated = no_calculated_raw = o2_calculated = lastValue = nox = noxGain = 0
    inPurge = inSafe = coraw = noraw = saveSend = inStop = cancel = confSend = lowPower = healthSend = battLow = inProtect = network = newCal = humid = healthPacket = locationSend = safeToggle = send60 = send70 = bigNet = False
    prsAvgNum = 3
    prsUnits = ["psi", "mbar", "inh2o", "kpa"]
    location = ["na", "na", "na", "na", "na", "na"]
    save = ["", "", "", ""]
    health = ["", "", "", "", "", ""]
    cal = ""
    slope1 = tries = 0
    last_health = last_wake = last_normal = first = lasto2 = time.ticks_ms()
    lastHum = lastHum2 = -5000000
    slope = startHum = x = sscTemp = 0
    netTime = 2000
    try:
        loadCalibration()
    except Exception as e:
        print(e)
        screen.error("Cannot load calibration: " + str(e))
    tempOffset = o2offset
    tempGain = o2gain
    newo2 = (((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)*o2gain
    ctr = ctr2 = timedout = cogain = 0
    r = msg = None
    pump.setPercent('b', 0, '+')
    startCode = blockN = time.ticks_ms()
    screen.boot_progress("Boot Successful!", 100)
    screen.change_page("home")
    temporary = pressure.stream_size
    err = None
    inStop = True
    tmcError = coError = noError = o2Error = sscError = bmeError = batError = senseError = screenError = sendError = mainError = cmdError = False
    screen.stop_mode()
    o2_sensor.get()
except Exception as e:
    print("Init Variables Failure")
try:
    cellCounts = adc.read()
    cellVolts = cellCounts * 0.00081
    if cellVolts < 3.0 and cellVolts > 2.95:
        screen.notify("Coin Cell Voltage Low!", "{}{}%".format("Replace Soon!\r\nVoltage: ", round(cellVolts, 1)))
        screen.write_global("tmBar.en", 1)
    elif cellVolts <= 2.85:
        screen.notify("Replace Coin Cells!", "{}{}%".format("Battery Level Critical!\r\nVoltage: ", round(cellVolts, 1)))
        screen.write_global("tmBar.en", 0)
except Exception as e:
    print("Cannot read coin cell voltage: ", e)

binary = 0
while True:
    try:
        try:
            if tmc_diag.value() == 1:
                try:
                    diagData = str(pump.getDiag())
                except Exception as e:
                    print("Diag read error: ", e)
                    diagData = "Unknown"
                try:
                    pump.setPercent('b', 0, '+')
                    pump.setPercent('a', 0, '+')
                    reset_tmc()
                except Exception as e:
                    print("Reset TMC error: ", e)
        except Exception as e:
            print("TMC Diag error: ", e)
        try:
            if tmcError is False and network is True:
                tmcError = True
                msg = "{} {} {}".format("E", "TMC Error:", str(diagData))
                xbee.txData(COORDINATOR_ADDR, msg)
                time.sleep(0.1)
        except Exception as e:
            print("Cannot send error packet: ", e)
        try:
            if newCal == True:
                newCal = False
                loadCalibration()
        except:
            pass
        #===================================================== Sense Stuff =================================================================
        try:
            cmd_handler()
            try:
                inline_temp = bme.read_temperature()/100
                inline_pressure = (bme.read_pressure()/25600)/2.4908890833333
                inline_humidity = int(bme.read_humidity()/1024)
                bmeError = False
            except Exception as e:
                if bmeError is False:
                    bmeError = True
                    err = "BME error" + str(e)
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
                sscTemp = prs_sensor.get_temp()
                p = prs_sensor.get()
                if p > -0.1 and p < 0.1:
                    p = 0.0
                pressure.add_point(p)
                p_inh2o = round(pressure.average(),1)
                p_unit = round(pressure.average()*prs_sensor.conversion_multiplier, 1)
                p_unit_noavg = round(p*prs_sensor.conversion_multiplier, 1)
                if abs(p_inh2o - lastValue) > 5:
                    slope+=1
                    if slope >= 20:
                        slope = 0
                        temporary = pressure.stream_size
                        pressure.stream_size = 1
                        p = prs_sensor.get()
                        if p > -0.1 and p < 0.1:
                            p = 0.0
                        pressure.add_point(p)
                        p_inh2o = round(pressure.average(),1)
                        p_unit = round(pressure.average()*prs_sensor.conversion_multiplier, 1)
                        p_unit_noavg = round(p*prs_sensor.conversion_multiplier, 1)
                else:
                    slope=0
                    pressure.stream_size = temporary
                lastValue = p
                sscError = False
            except Exception as e:
                if sscError is False:
                    sscError = True
                    err = "SSC Error: " + str(e)
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
                co = ec_co_sensor.readGas()
                if 30 < sscTemp < 35:
                    co-=10
                elif 35 <= sscTemp < 40:
                    co-=10
                elif 40 <= sscTemp < 45:
                    co-=10
                co = round((((co - ec_co_sensor.raw_Low) * (ec_co_sensor.ref_High - ec_co_sensor.ref_Low)) / (ec_co_sensor.raw_High - ec_co_sensor.raw_Low)) + ec_co_sensor.ref_Low)
                if co < 0: co = 0
                coError = False
            except Exception as e:
                if coError is False:
                    coError = True
                    err = "CO Read Error: " + str(e)
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
                no = ec_no_sensor.readGas()
                if 30 < sscTemp < 35:
                    no-=5
                elif 35 <= sscTemp < 40:
                    no-=10
                elif 40 <= sscTemp < 45:
                    no-=15
                elif 45 <= sscTemp < 52:
                    no-=23
                elif 52 <= sscTemp < 55:
                    no-=28
                elif 55 <= sscTemp < 57:
                    no-=70
                elif 57 <= sscTemp < 60:
                    no-=90
                elif 60 <= sscTemp < 62:
                    no-=130
                elif 62 <= sscTemp < 63:
                    no-=150
                elif 63 <= sscTemp < 64:
                    no-=170
                elif 64 <= sscTemp < 65:
                    no-=180
                elif sscTemp >= 65:
                    no-=200
                no = round((((no - ec_no_sensor.raw_Low) * (ec_no_sensor.ref_High - ec_no_sensor.ref_Low)) / (ec_no_sensor.raw_High - ec_no_sensor.raw_Low)) + ec_no_sensor.ref_Low)
                if no < 0: no = 0
                nox = no*noxGain
                noError = False
            except Exception as e:
                if noError is False:
                    noError = True
                    err = "NO Read Error: " + str(e)
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
                batError = False
            except Exception as e:
                if batError is False:
                    battError = True
                    voltage = 5.0
                    current = 999
                    percent = 100
                    bTemp = 99
                    err = "Battery error" + str(e)
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
                if time.ticks_diff(time.ticks_ms(), lasto2) >= 1000:
                    lasto2 = time.ticks_ms()
                    o2_sensor.get()
                newo2 = (((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)*o2gain
                if newo2 < 0 : newo2 = 0
                oxy.add_point(newo2)
                o2Error = False
            except Exception as e:
                if o2Error is False:
                    o2Error = True
                    err = "O2 Sensor Read Error" + str(e)
                    screen.error(err)
                    try:
                        if network is True:
                            msg = '{} {}'.format("E", err)
                            xbee.txData(COORDINATOR_ADDR, msg)
                            time.sleep(0.1)
                    except Exception as e:
                        print("Cannot send error packet: ", e)
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
            if button.value() == 1:
                if check_button(button) == 'boot':
                    shutdown()   
            cmd_handler()
            if inProtect is False:
                if inStop is False:
                    screen.send_CO(co_calculated)
                else:
                    screen.write_global_text("home", "tCO", "STOP" )
            if cal == "co":
                screen.cal_CO(ec_co_sensor.raw_ppm)
            
            if inProtect is False:
                if inStop is False:
                    screen.send_NO(int(nox))
                else:
                    screen.write_global_text("home", "tNOX", "----" )
            if cal == "no":
               screen.cal_NO(ec_no_sensor.raw_ppm)
            screen.write_global_text("pumpsCal", "tPumpCO",  int(co))
            screen.write_global_text("calibration", "tPumph",  round(coGain, 3))
            screen.write_global_text("calibration", "tNoxMult",  round(noxGain, 2))
            cmd_handler()
            if cal == "None" or cal == "" or cal is None:
                screen.send_PR(p_unit_noavg, p_unit)
            cmd_handler()
            if inProtect is False:
                if inStop is False:
                    screen.send_O2(round(newo2, 1))
                else:
                    screen.write_global_text("home", "tO2", "--.-" )
            if cal == "o2":
                screen.cal_O2(round(o2_sensor.o2, 2))
                screen.write_global_text("o2Cal", "tO2Calc", str(round(oxy.average(), 2)))
                screen.write_global_text("o2Cal", "tOff", str(round(o2offset, 4))) 
                screen.write_global_text("o2Cal", "tGain", str(round(o2gain, 4))) 
            cmd_handler()
            if co >= 4000 and override == 0:                  #PROTECT MODE
                inProtect = True
                inSafe = True
            elif co >= 1000 and co < 4000 and override == 0:    #SAFE MODE
                inProtect = False
                inSafe = True
            elif co >= 250 and co < 1000 and override == 0:    #SAFE OR RUN MODE
                inProtect = False
            elif co < 250 or override == 1:                    #RUN OR STOP MODE
                inProtect = False
                if safeToggle == False:
                    inSafe = False


            if time.ticks_diff(time.ticks_ms(), lastHum2) > 20000 and time.ticks_diff(time.ticks_ms(), startCode) > 10000:
                if inline_humidity >= 80 and inProtect is False and inSafe is False: #Stop pump if not purging or safe mode, could have humid ambient air purging in
                    inStop = True
                    humid = True
                    screen.set_desiccant(1)
                    if time.ticks_diff(time.ticks_ms(), lastHum) > 180000:
                        lastHum = time.ticks_ms()
                        screen.notify("Desiccant is Saturated!", "{}{}%".format("Sense pump stopped!\r\nHumidity: ", int(inline_humidity)))
                        screen.write_global("tmBar.en", 0)
                    cmd_handler()
                elif inline_humidity >= 70 and inline_humidity < 80:
                    screen.set_desiccant(1)
                    if send70 == False:
                        send70 = True
                        screen.notify("70% Humidity Warning!", "{}{}%".format("Sense pump stops at 80%.\r\nHumidity: ", int(inline_humidity)))
                        screen.write_global("tmBar.en", 1)
                    cmd_handler()
                elif inline_humidity >= 60 and inline_humidity < 70:
                    screen.set_desiccant(1)
                    if send60 == False:
                        send60 = True
                        screen.notify("60% Humidity Warning!", "{}{}%".format("Sense pump stops at 80%.\r\nHumidity: ", int(inline_humidity)))
                        screen.write_global("tmBar.en", 1)
                    cmd_handler()
                elif inline_humidity >= 55 and inline_humidity < 60:
                    screen.set_desiccant(0)
                elif inline_humidity < 55:
                    send60 = False
                    send70 = False
                    screen.set_desiccant(0)


            cmd_handler()
            if inProtect is True and safeToggle is False:
                if status != 3:
                    status = 3
                    screen.protect_mode()
                    pump.setPercent('a', 100, '+') #1.04 l/min
                    pump.setPercent('b', 0, '+')  #0.36 l/min
                co_calculated = co_calculated_raw = 10000
            elif safeToggle is True and override == 0:
                if status != 1:
                    status = 1
                    screen.safe_mode()
                if inStop is True:     #SAFE MODE, WHILE IN STOP MODE
                    pump.setPercent('a', 100, '+')
                    pump.setPercent('b', 0, '+')
                    co_calculated = co_calculated_raw = 0
                elif inStop is False:                  #SAFE MODE, WHILE IN RUN MODE
                    pump.setPercent('a', 100, '+') #PURGE - 1.04 l/min
                    pump.setPercent('b', 100, '+') #SENSE - 0.36 l/min
                    co_calculated = co*(coGain)
                    co_calculated_raw = int(ec_co_sensor.raw_ppm)*(coGain)
            elif inSafe is True:
                if status != 1:
                    status = 1
                    screen.safe_mode()
                if inStop is True:     #SAFE MODE, WHILE IN STOP MODE
                    pump.setPercent('a', 100, '+')
                    pump.setPercent('b', 0, '+')
                    co_calculated = co_calculated_raw = 0
                elif inStop is False:                  #SAFE MODE, WHILE IN RUN MODE
                    pump.setPercent('a', 100, '+')
                    pump.setPercent('b', 100, '+')
                    co_calculated = co*(coGain)
                    co_calculated_raw = int(ec_co_sensor.raw_ppm)*(coGain)
            elif inSafe is False and inProtect is False:
                if inStop is True:
                    if status != 4:
                        screen.stop_mode()
                        status = 4
                        pump.setPercent('a', 0, '+')
                        pump.setPercent('b', 0, '+')
                    co_calculated = co_calculated_raw = 0
                elif inStop is False:
                    if status != 0:
                        status = 0
                        screen.run_mode()
                        pump.setPercent('a', 0, '+')
                        pump.setPercent('b', 100, '+')
                    co_calculated = co
                    co_calculated_raw = ec_co_sensor.raw_ppm
            cmd_handler()
            screen.send_admin(round(-1000*current, 2), round(voltage, 2), int(ec_co_sensor.raw_ppm), int(co), int(ec_no_sensor.raw_ppm), int(nox),round(o2_sensor.o2, 1), int(inline_humidity), round(inline_temp, 1), round(inline_pressure, 1), round(cellVolts, 2), round(sscTemp, 1))
            screenError = False
        except TypeError as t:
            pass
        except Exception as e:
            if screenError is False:
                screenError = True
                err = "Screen Data Error: " + str(e)
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
            if locationSend is True:
                if network is True:
                    msg = '{} {} {} {} {} {} {}'.format("L", location[0], location[1], location[2], location[3], location[4], location[5])
                    xbee.txData(COORDINATOR_ADDR, msg)
                    screen.notify("Location Sent!", "")
                    screen.write_global("tmBar.en", 1)
                else:
                    screen.notify("Location not Sent!", "Try again.")
                    screen.write_global("tmBar.en", 1)
                locationSend = False
            cmd_handler()
            if healthSend is True:
                if network is True:
                    msg = '{} {} {} {} {} {} {}'.format("HR", health[0], health[1], health[2], health[3], health[4], health[5])
                    xbee.txData(COORDINATOR_ADDR, msg)
                    screen.notify("Health Report Sent!", "")
                    screen.write_global("tmBar.en", 1)
                else:
                    screen.notify("Health not Sent!", "Try again.")
                    screen.write_global("tmBar.en", 1)
                healthSend = False
            if saveSend is True:
                if network is True:
                    if save[0] == "pressure":
                        msg = '{} {} {} {}'.format("S", save[0], save[1], save[2])
                        xbee.txData(COORDINATOR_ADDR, msg)
                    elif save[0] == "flue":
                        msg = '{} {} {} {} {}'.format("S", save[0], save[1], save[2], save[3])
                        xbee.txData(COORDINATOR_ADDR, msg)
                    screen.notify("Save Prs/Flue Sent!", "")
                    screen.write_global("tmBar.en", 1)
                else:
                    screen.notify("Values not Sent!", "Try again.")
                    screen.write_global("tmBar.en", 1)
                saveSend = False
            cmd_handler()
            try:
                if time.ticks_diff(time.ticks_ms(), last_health) > 180000 or healthPacket is False: ## Send every 180 seconds or if it failed to send last loop cycle
                    last_health = time.ticks_ms()
                    if voltage <= 3.3 and voltage > 3.2:
                        screen.notify("Battery is low!", "Please replace soon.")
                        screen.write_global("tmBar.en", 1)
                        battLow = False
                    elif voltage <= 3.2:
                        battLow = True
                        screen.change_page("deadBatt")
                        time.sleep(10)
                        shutdown()
                    if sscTemp >= 60 and sscTemp < 70:
                        screen.notify("Temperature Alert!", "{}{}C".format("CO/NOx/O2 lose accuracy!\r\nTemperature: ", round(sscTemp, 1)))
                        screen.write_global("tmBar.en", 1)
                    elif sscTemp >= 70:
                        screen.notify("Device Overheated!", "Shutting Down...")
                        screen.write_global("tmBar.en", 0)
                        time.sleep(5)
                        shutdown()
                    if bigNet == False:
                        msg = '{} {} {} {} {}'.format("SH", round(inline_temp, 1), int(inline_humidity), inline_pressure, round(sscTemp,1))
                        xbee.txData(COORDINATOR_ADDR, msg)
                        msg = '{} {} {} {} {} {}'.format("BH", round(voltage, 2), round(-1*current, 2), round(percent,1), round(bTemp,1),  round(cellVolts, 2))
                        xbee.txData(COORDINATOR_ADDR, msg)
                        msg = '{} {} {}'.format("OH", round(o2_sensor.ppo2,1), round(o2_sensor.pressure,1)) 
                        xbee.txData(COORDINATOR_ADDR, msg)
                        msg = '{} {} {} {} {} {} {} {} {} {} {} {} {} {}'.format("CF", int(ec_co_sensor.raw_Low), int(ec_co_sensor.ref_Low), int(ec_co_sensor.raw_High), int(ec_co_sensor.ref_High), int(ec_no_sensor.raw_Low), int(ec_no_sensor.ref_Low), int(ec_no_sensor.raw_High), int(ec_no_sensor.ref_High), round(o2offset, 4), round(o2gain, 4), round(prs_sensor.zero_offset, 3), round(coGain,3), round(noxGain,2))
                        xbee.txData(COORDINATOR_ADDR, msg)
                    healthPacket = True
            except Exception as e:
                healthPacket = False
                print(e)
            if bigNet == False:
                if time.ticks_diff(time.ticks_ms(), last_normal) > 1000: ## 1 seconds
                    last_normal = time.ticks_ms()
                    if inStop is False:
                        msg = '{} {} {} {} {} {} {} {}'.format("N", round(oxy.average(),1), int(co_calculated), round(ec_co_sensor.raw_ppm,1), int(nox), round(ec_no_sensor.raw_ppm,1), round(p_inh2o,1), int(status))
                        time.sleep(0.005)
                        xbee.txData(COORDINATOR_ADDR, msg)
                    elif inStop is True:
                        msg = '{} {} {}'.format("MP", round(p_inh2o,1), int(status))
                        xbee.txData(COORDINATOR_ADDR, msg)
            else:
                if time.ticks_diff(time.ticks_ms(), last_normal) > netTime:
                    last_normal = time.ticks_ms()
                    if inStop is False:
                        msg = '{} {} {} {} {} {} {} {}'.format("N", round(oxy.average(),1), int(co_calculated), round(ec_co_sensor.raw_ppm,1), int(nox), round(ec_no_sensor.raw_ppm,1), round(p_inh2o,1), int(status))
                        time.sleep(0.005)
                        xbee.txData(COORDINATOR_ADDR, msg)
                    elif inStop is True:
                        msg = '{} {} {}'.format("MP", round(p_inh2o,1), int(status))
                        xbee.txData(COORDINATOR_ADDR, msg)
            cmd_handler()
            sendError = False
        except TypeError as t:
            pass
        except Exception as e:
            if sendError is False:
                sendError = True
                err = "XBee Sending Error: " + str(e)
                print(err)
                screen.error(err)
                try:
                    if network is True:
                        msg = '{} {}'.format("E", err)
                        xbee.txData(COORDINATOR_ADDR, msg)
                        time.sleep(0.1)
                except Exception as e:
                    print("Cannot send error packet: ", e)
        gc.collect()
        mainError = False
    except Exception as e:
        raise e
        if mainError is False:
            mainError = True
            err = "Main Loop Runtime Error: " + str(e)
            print(err)
            try:
                if network is True:
                    msg = '{} {}'.format("E", err)
                    xbee.txData(COORDINATOR_ADDR, msg)
                    time.sleep(0.1)
            except Exception as e:
                print("Cannot send error packet: ", e)
