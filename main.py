import time
import machine
import stm
import pyb
from machine import Pin, I2C, UART
from pyb import Pin, LED
import uasyncio
import json

# BOOT UP
AWAKE_START = time.ticks_ms()
button = Pin('A1', Pin.IN, pull=Pin.PULL_DOWN)
pump_uart = machine.UART(3,115200)
en5 = Pin('C9', Pin.OUT_PP)
en5.value(0)
en3v3 = Pin('A15', Pin.OUT_PP)
en3v3.value(0)

def check_button(b):
    a = 0
    start_time = time.ticks_ms()
    hold_ms = 1000
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
    else:
        if time.ticks_diff(time.ticks_ms(), AWAKE_START) < 10000:
            pass
        else:
            stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
            machine.deepsleep()

en5.value(1)

# INITIAL CONFIG OF ANALYZER
import O2_Luminox as O2_Luminox
import PRS_TruStabilitySSC as PRS_TruStabilitySSC
import aducm355 as Electrochem
from Nextion import PSNERGY_Screen as Screen
import xbee as xbee
import measurements as measurements
import TMC7300 as tmc
from BME280 import BME280
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

time.sleep(1)

#screen setup
try:
    nextion = machine.UART(6, 115200)
    nextion.init(baudrate=115200, bits=8, parity=None, stop=1, timeout_char=1, timeout=3)
    screen = Screen(nextion, debug_mode=False)
    time.sleep(0.1)
    screen.set_device_name(str(DEVICE_ID))
    screen.boot_progress("Initializing Screen...", 20)
except Exception as e:
    print("Screen Initialization error: ", e)
    screen.boot_progress("Screen Initialization Failure!", 20)
time.sleep(0.01)

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
time.sleep(0.01)

#t/h setup
try:
    screen.boot_progress("Initializing Temperature Sensor...", 40)
    bme = BME280(mode=1, address=0x76, i2c=machine.I2C(2))
except Exception as e:
    print("Temp Initialization Failure: ", e)
    screen.boot_progress("Temperature Sensor Initialization Failure!", 40)
time.sleep(0.01)

try:
    screen.set_battery_percent(int(battery.batt_percent()))
except Exception as e:
    print(e)

#pressure setup
try:
    screen.boot_progress("Initializing Pressure Sensor...", 50)
    while True:
        time.sleep(0.05)
        prs_sensor = PRS_TruStabilitySSC.SSC(0x28, 2, -138.5379, 138.5379)
        pressure = measurements.Measurement()
        pressure.create_stream(30, 4)
        break
except Exception as e:
    print("Pressure Initialization Failure: ", e)
    screen.boot_progress("Pressure Sensor Initialization Failure!", 50)

time.sleep(0.01)
        
#xbee setup
try:
    screen.boot_progress("Initializing XBee...", 60)
    xbee.DEVICE = machine.UART(2, 115200)
    xbee.ID = DEVICE_ID
    screen.set_signal_strength(2)
    time.sleep(0.05)
except Exception as e:
    print("Xbee Initialization Failure: ", e)
    screen.boot_progress("XBee Initialization Failure!", 60)

time.sleep(0.01)

# Pump setup
try:
    screen.boot_progress("Initializing Pumps...", 70)
    pumpSTDBY = pyb.Pin(pyb.Pin.board.C1, Pin.OUT_PP, pull=Pin.PULL_DOWN)
    pumpSTDBY.value(0)
    pumpEN = pyb.Pin(pyb.Pin.board.C3, Pin.OUT_PP, pull=Pin.PULL_DOWN)
    pumpEN.value(0)
    #en3v3.value(1)
    #everything 3v3 off
    pumpSTDBY.value(0)
    pumpEN.value(0)
    en3v3.value(0)
    
    #start motor sequence
    pump = tmc.TMC_UART(pump_uart, 0x00)
    en3v3.value(1)
    pumpSTDBY.value(1)
    time.sleep(0.01)
    
    #setup motor chip for dual motors
    pump.write_reg(tmc.GCONF, [0x00,0x00,0x00,0x01])
    #set current limit to max on both motors
    pump.write_reg(tmc.CURRENT_LIMIT, [0x00,0x00,0x1f,0x01])
    #set inital motor speeds and enable
    pump.setPercent('a', 0, '+')
    pump.setPercent('b', 0, '+')
    pumpEN.value(1)
    en3v3.value(1)
    pump_saved = 100
except Exception as e:
    print("Pump Initialization Failure: ", e)
    screen.boot_progress("Pump Initialization Failure!", 70)

time.sleep(0.01)

def loadCalibration():  #load values from config text into local variables
    global ec_co_sensor, ec_no_sensor, o2low, o2targetlow, o2high, o2targethigh, o2offset, o2gain, prs_sensor
    try:
        config = list(range(11))
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
        return "Done loading"
    except Exception as e:
        err = "EC Calibration Load Error: " + str(e)
        print(err)
        screen.error(err)

##analyzer setup
try:
    screen.boot_progress("Initializing Oxygen Sensor...", 80)
    o2_uart = machine.UART(5, 9600)
    o2_sensor = O2_Luminox.Luminox(o2_uart, o2_offset=float(Configuration["Calibration"]["luminox_offset"]))
    oxy = measurements.Measurement()
    oxy.create_stream(2, 1)
    time.sleep(.01)
except Exception as e:
    print("O2 Initialization Failure: ", e)
    screen.boot_progress("O2 Initialization Failure!", 80)
screen.boot_progress("Initializing Electrochemical Sensors...", 90)
try:
    ec_co_sensor = Electrochem.CO(addr=76, bus=i2c_bus)
    ec_co_sensor.configure_sensor(
        vbias=Configuration["Electrochem"]["CO"]["vbias"],
        vzero_voltage=Configuration["Electrochem"]["CO"]["vzero_voltage"],
        sensor_sens=Configuration["Electrochem"]["CO"]["sensor_sens"],
        tia_gain=Configuration["Electrochem"]["CO"]["tia_gain"],
        rload=Configuration["Electrochem"]["CO"]["rload"]
    )
except Exception as e:
    print("CO Initialization Failure: ", e)
    screen.boot_progress("CO Sensor Initialization Failure!", 90)
try:
    ec_no_sensor = Electrochem.NO(addr=77, bus=i2c_bus)
    ec_no_sensor.configure_sensor(
        vbias=Configuration["Electrochem"]["NO"]["vbias"],
        vzero_voltage=Configuration["Electrochem"]["NO"]["vzero_voltage"],
        sensor_sens=Configuration["Electrochem"]["NO"]["sensor_sens"],
        tia_gain=Configuration["Electrochem"]["NO"]["tia_gain"],
        rload=Configuration["Electrochem"]["NO"]["rload"]
    )
except Exception as e:
    print("NO Initialization Failure: ", e)
    screen.boot_progress("NO Sensor Initialization Failure!", 90)
try:
    ec_co_sensor.setCalValues(
        raw_Low     = int(Configuration["Electrochem"]["CO"]["raw_Low"]),
        raw_High    = int(Configuration["Electrochem"]["CO"]["raw_High"]),
        ref_Low     = int(Configuration["Electrochem"]["CO"]["ref_Low"]),
        ref_High    = int(Configuration["Electrochem"]["CO"]["ref_High"])
    )
except Exception as e:
    print("CO calibration set failure: ", e)
    screen.boot_progress("CO Calibration Failure!", 90)
try:
    ec_no_sensor.setCalValues(
        raw_Low     = int(Configuration["Electrochem"]["NO"]["raw_Low"]),
        raw_High    = int(Configuration["Electrochem"]["NO"]["raw_High"]),
        ref_Low     = int(Configuration["Electrochem"]["NO"]["ref_Low"]),
        ref_High    = int(Configuration["Electrochem"]["NO"]["ref_High"])
    )
except Exception as e:
    print("NO calibration set failure: ", e)
    screen.boot_progress("NO Calibration Failure!", 90)
try:
    ec_co = measurements.Measurement()
    ec_co.create_stream(2, 0)
except Exception as e:
    print("CO Measurement Initialization Failure: ", e)
    screen.boot_progress("CO Measurement Initialization Failure!", 90)
try:
    ec_no = measurements.Measurement()
    ec_no.create_stream(2, 0)
except Exception as e:
    print("NO Measurement Initialization Failure: ", e)
    screen.boot_progress("NO Measurement Initialization Failure!", 90)
try:
    if Configuration["XbeeConfig"]["tx_co_config"] == 1:
        xbee.send(
                    "CC",
                    int(Configuration["Electrochem"]["CO"]["vbias"]),
                    int(Configuration["Electrochem"]["CO"]["vzero_voltage"]),
                    int(Configuration["Electrochem"]["CO"]["sensor_sens"]),
                    int(Configuration["Electrochem"]["CO"]["tia_gain"]),
                    int(Configuration["Electrochem"]["CO"]["rload"]),
                    int(ec_co_sensor.raw_Low),
                    int(ec_co_sensor.raw_High),
                    int(ec_co_sensor.ref_Low),
                    int(ec_co_sensor.ref_High)
                    )
except Exception as e:
    print("XBee Send CO Config Failure: ", e)
    screen.boot_progress("XBee Send CO Configuration Failure!", 90)
try:
    screen.set_device_name(str(DEVICE_ID))
except Exception as e:
    print("Set boot after EC Failure: ", e)

def writeCalVals():
    global ec_co_sensor, ec_no_sensor,o2low,o2high,o2targetlow,o2targethigh, screen, prs_sensor, o2offset, o2gain, prs_sensor
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
        f.write(str(prs_sensor.zero_offset))
        f.close()
    except Exception as e:
        err = "Write Calibration Values to File Error: " + str(e)
        print(err)
        screen.error(err)

def cmd_handler():
    global screen, pump, pump_saved, inPurge, inSafe, pressure, prs_sensor, prsUnits, location, save, oxy, co, o2low, o2high, o2targetlow, o2targethigh
    global ec_co_sensor, ec_no_sensor, coraw, noraw, status, inStop, cal, cancel, o2gain, o2offset, locationSend, saveSend, health, healthSend, p_inh2o
    instr = ""
    state = None
    try:
        instr = screen.read_instr()
        if instr != "" and instr != None and instr != '':
            if instr[0] == "pump":
                pump_saved = int(instr[1])
                pump.setPercent('b', pump_saved, '+')
            elif instr[0] == "state":
                if instr[1] == "safe":
                    inSafe = True
                    inStop = False
                    status = 1
                    screen.safe_mode()
                    pump.setPercent('b', 50, '+')
                    pump.setPercent('a', 100, '+')
                    screen.notify("Safe Mode Engaged!", "")
                elif instr[1] == "run":
                    inSafe = False
                    inStop = False
                    status = 0
                    #screen.run_mode()
                    pump.setPercent('b', pump_saved, '+')
                elif instr[1] == "stop":
                    inStop = True
                    inSafe = False
                    #screen.stop_mode()
                    status = 3
                    pump.setPercent('b', 0, '+')
                    #screen.notify("Stop Mode Engaged!", "Sense pump not active.\r\nPurge pump active.")
            elif instr[0] == "moving":
                pressure.stream_size = int(instr[1])
                screen.notify("Prs Average Changed!", "{}{}".format("New Average: ", int(instr[1])))
            elif instr[0] == "unit":
                prs_sensor.change_unit(prsUnits[int(instr[1])])
                screen.notify("Prs Units Changed!", "{}{}".format("New Units: ", prsUnits[int(instr[1])]))
            elif instr[0] == "calvals":
                screen.populate_calvals(o2gain, o2offset, ec_co_sensor.raw_High, ec_co_sensor.raw_Low, ec_no_sensor.raw_High, ec_no_sensor.raw_Low, prs_sensor.zero_offset)
            elif instr[0] == "location":
                location = instr[1:]
                locationSend = True
            elif instr[0] == "save":    # "save flue initial", "save pressure final"
                if instr[1] == "pressure":
                    save[0] = "pressure"
                    save[1] = str(pressure.average())
                    save[2] = str(instr[2])
                    save[3] = ""
                elif instr[1] == "flue":
                    save[0] = "flue"
                    save[1] = str(oxy.average())
                    save[2] = str(co)
                    save[3] = str(instr[2])
                saveSend = True
            elif instr[0] == "report":
                health = instr[1:]
                healthSend = True
            elif instr[0] == "zero":
                screen.notify("Zeroing Pressure...", "Wait for finish notification!")
                loadCalibration()
                prs_sensor.zero()
                writeCalVals()
                screen.notify("Pressure Offset Saved!", "")
            elif instr[0] == "cal":
                cal = str(instr[1])
            elif instr[0] == "o2offset":
                loadCalibration()
                o2offset = (o2_sensor.ppo2/o2_sensor.pressure)*100
                writeCalVals()
            elif instr[0] == "o2gain":
                loadCalibration()
                o2gain = 20.7/(((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)
                writeCalVals()
                screen.change_page("secretCal")
                screen.notify("Step 2 Success!", "{}\r{}".format("O2 gain has been created.", "\r\nCalibration successful!"))
            elif instr[0] == "coLow":
                loadCalibration()
                ec_co_sensor.ref_Low = int(instr[1])
                ec_co_sensor.raw_Low = int(instr[2])
                writeCalVals()
                screen.notify("CO Low Saved!", "")
            elif instr[0] == "coHigh":
                loadCalibration()
                ec_co_sensor.ref_High = int(instr[1])
                ec_co_sensor.raw_High = int(instr[2])
                writeCalVals()
                screen.notify("CO High Saved!", "")
            elif instr[0] == "noLow":
                loadCalibration()
                ec_no_sensor.ref_Low = int(instr[1])
                ec_no_sensor.raw_Low = int(instr[2])
                writeCalVals()
                screen.notify("NO Low Saved!", "")
            elif instr[0] == "noHigh":
                loadCalibration()
                ec_no_sensor.ref_High = int(instr[1])
                ec_no_sensor.raw_High = int(instr[2])
                writeCalVals()
                screen.notify("NO High Saved!", "")
            elif instr[0] == "cancel":
                cancel = True
            elif instr[0] == "dnd":
                screen.notifications(bool(instr[1]))
            else:
                err = ("command not recognized: " + str(instr))
                print(err)
                screen.error(err)
    except Exception as e:
        err = "Instruction Reading Failure: " + instr
        print(err)
        screen.error(err)

def shutdown():
    global button, pump, co, avgCO, ec_co_sensor, ec_co, en3v3, en5, cancel, no, avgNO, ec_no_sensor, ec_no, battLow
    try:
        beginPurge = time.time()
        pump.setPercent('a', 100, '+')
        pump.setPercent('b', 0, '+')
        cancel = False
        if battLow is False:
            screen.change_page("shutdown")
        while co > 20: #30 seconds Purge max
            screen.send_shutdown("CO Too High!", "CO Target: <20ppm", co)
            cmd_handler()
            if time.time() - beginPurge > 30:
                break
            elif cancel is True:
                cancel = False
                screen.change_page("home")
                screen.notify("Shutdown Canceled!")
                return -1
            else:
                cmd_handler()
                avgCO = ec_co_sensor.readGas()
                ec_co.add_point(avgCO)
                co = ec_co.average()
            cmd_handler()
        pump.setPercent('b', 0, '+')
        pump.setPercent('a', 0, '+')
        en3v3.value(0)
        en5.value(0)
        time.sleep(3)
        stm.mem32[stm.PWR + stm.PWR_CSR] |= 1 << 8 # enable WKUP pin on PA0
        machine.deepsleep()
    except Exception as e:
        print("Shutdown Error: ", e)

try:
    o2gain = 0
    o2offset = 0
    avgCO = p_inh2o = p_unit = lastCO = newo2 = p_unit_noavg = current = percent = voltage = mcu_temp = ptemp = inline_pressure = co = no = status = 0
    co_calculated = co_calculated_raw = no_calculated = no_calculated_raw = o2_calculated = 0
    inPurge = inSafe = coraw = noraw = saveSend = inStop = cancel = locationSend = lowPower = healthSend = battLow = sendMAX = network = False
    prsAvgNum = 3
    prsUnits = ["psi", "mbar", "inh2o", "kpa"]
    location = ["", "", "", "", "", ""]
    save = ["", "", "", ""]
    health = ["", "", "", "", "", ""]
    cal = ""
    slope1 = 0
    last_health = last_normal = batt = time.ticks_ms()
    slope = 0
    startHum = 0
    x=0
    ctr = ctr2 = timedout = 0
    r = None
    beginStartLoop = time.ticks_ms()
    loadCalibration()
    screen.boot_progress("Boot Successful!", 100)
    time.sleep(0.5)
    screen.change_page("home")
except Exception as e:
    err = "Init Variables Failure: " + str(e)
    print(err)
    screen.error(err)

while True:
    try:
        #===================================================== Sense Stuff =================================================================
        try:
            
            cmd_handler()
            try:
                avgCO = ec_co_sensor.readGas()
                print(avgCO)
                ec_co.add_point(avgCO)
                co = ec_co.average()
                cmd_handler()
            except Exception as e:
                err = "CO Read Error: " + str(e)
                print(err)
                screen.error(err)
            cmd_handler()
            try:
                avgNO = ec_no_sensor.readGas()
                print(avgNO)
                ec_no.add_point(avgNO)
                no = ec_no.average()
                cmd_handler()
            except Exception as e:
                err = "NO Read Error: " + str(e)
                print(err)
                screen.error(err)
            cmd_handler()
            try:
                p = prs_sensor.get()
                pressure.add_point(p)
                p_inh2o = round(pressure.average(),1)
                p_unit = round(pressure.average()*prs_sensor.conversion_multiplier, prs_sensor.rounder)
                p_unit_noavg = round(p*prs_sensor.conversion_multiplier, prs_sensor.rounder)
            except Exception as e:
                err = "Pressure Read Error: " + str(e)
                print(err)
                screen.error(err)
            slope += 1
            cmd_handler()
            try:
                inline_temp = bme.read_temperature()/100
            except Exception as e:
                err = "Inline Temperature Read Error: " + str(e)
                print(err)
                screen.error(err)
            try:
                inline_humidity = bme.read_humidity()/1000
                if int(inline_humidity) > 50 and int(inline_humidity) < 80:
                    screen.set_desiccant(1) #Moist
                elif int(inline_humidity) >= 80:
                    screen.set_desiccant(1)
                    if x > 10:
                        x = 0
                        screen.notify("Desiccant is Saturated!", "{}{}%".format("Replace Immediately!\r\nHumidity: ", int(inline_humidity)))
                    x += 1
                else:
                    screen.set_desiccant(0) #Dry
            except Exception as e:
                err = "Inline Humidity Read Error: " + str(e)
                print(err)
                screen.error(err)
            cmd_handler()
            try:
                inline_pressure = bme.read_pressure()
            except Exception as e:
                err = "Inline Pressure Read Error: " + str(e)
                print(err)
                screen.error(err)
            try:
                voltage = battery.oc_voltage()
                current = battery.current_draw()
                percent = battery.batt_percent()
                screen.set_battery_percent(int(percent))
                cmd_handler()
            except Exception as e:
                err = "Battery Health Read Failure: " + str(e)
                print(err)
                screen.error(err)
                voltage = 5.0
            cmd_handler()
            try:
                if voltage <= 3.2 and voltage > 3.0 and time.ticks_diff(time.ticks_ms(), batt) > 180000:  #Send notification every 3 mins
                    batt = time.ticks_ms()
                    screen.notify("Battery is low!", "")
                elif voltage <= 3.0:
                    battLow = True
                    screen.change_page("deadBatt")
                    time.sleep(10)
                    shutdown()
            except Exception as e:
                err = "Low power error: " + str(e)
                print(err)
                screen.error(err)
        except Exception as e:
            err = "Sensing Thread Failure: " + str(e)
            print(err)
            screen.error(err)
        cmd_handler()
        #======================================== Screen Stuff =======================================================================
        try:
            if button.value() ==1:
                if check_button(button) == 'boot':
                    shutdown()
        except Exception as e:
            err = "Sleep Mode Failure: " + str(e)
            print(err)
            screen.error(err)
        if time.ticks_diff(time.ticks_ms(), beginStartLoop) > 25000:
            try:  
                if sendMAX is False:
                    if inStop is False:
                        screen.send_CO(co_calculated)
                    else:
                        screen.write_global_text("home", "tCO", "STOP" )
                if cal == "co":
                    screen.cal_CO(ec_co_sensor.raw_ppm)
            except Exception as e:
                err = "Write CO to Screen Failure: " + str(e)
                print(err)
                screen.error(err)
            cmd_handler()
            try:
                if sendMAX is False:
                    if inStop is False:
                        screen.send_NO(int(no))
                    else:
                        screen.write_global_text("home", "tNOX", "----" )
                if cal == "no":
                   screen.cal_NO(ec_no_sensor.raw_ppm)
            except Exception as e:
                err = "Write NO to Screen Failure: " + str(e)
                print(err)
                screen.error(err)
        else:
            if inStop is False:
                screen.write_global_text("home", "tCO", "----" )
                screen.write_global_text("home", "tNOx", "----" )
        cmd_handler()
        try:
            if cal == "None" or cal == "" or cal is None:
                screen.send_PR(p_unit_noavg, p_unit)
        except Exception as e:
            err = "Write Pressure to Screen Failure: " + str(e)
            print(err)
            screen.error(err)
        try:
            if sendMAX is False:
                if inStop is False:
                    screen.send_O2(round(newo2, 1))
                else:
                    screen.write_global_text("home", "tO2", "--.-" )
            if cal == "o2":
                screen.cal_O2(round(o2_sensor.o2, 2))
        except Exception as e:
            err = "Write O2 to Screen Failure: " + str(e)
            print(err)
            screen.error(err)     
        cmd_handler()
        if ((co >= 2000 and co < 3000) or co - lastCO > 100):
            try:
                sendMAX = False
                inPurge = True
                status = 2
                screen.purge_mode()
                if inStop is True:     #PURGE MODE, WHILE IN STOP MODE
                    pump.setPercent('a', pump_saved, '+')
                    pump.setPercent('b', 0, '+')
                else:                  #PURGE MODE, WHILE IN RUN MODE
                    pump.setPercent('a', pump_saved, '+')
                    pump.setPercent('b', pump_saved, '+')
                co_calculated = int(co)*2
                co_calculated_raw = int(ec_co_sensor.raw_ppm)*2
                cmd_handler()
            except Exception as e:
                err = "Purge stage one failure: " + str(e)
                print(err)
                screen.error(err)
        elif (co >= 3000 or co - lastCO > 250):   #PROTECT MODE, DOESNT MATTER FOR RUN/STOP MODE
            try:
                sendMAX = True
                inPurge = True
                status = 2
                screen.protect_mode()
                pump.setPercent('a', 100, '+')
                pump.setPercent('b', 0, '+')
                co_calculated = 8000
                co_calculated_raw = 8000
                cmd_handler()
            except Exception as e:
                err = "Purge stage two failure: " + str(e)
                print(err)
                screen.error(err)
        elif co >= 1500 and co < 2000:
            if inPurge is True:
                co_calculated = int(co)*2
                co_calculated_raw = int(ec_co_sensor.raw_ppm)*2
            elif inPurge is False:
                co_calculated = int(co)
                co_calculated_raw = int(ec_co_sensor.raw_ppm)
        elif co < 1500:
            try:
                sendMAX = False
                inPurge = False
                if inStop is True:       #NO PURGE, STOP MODE
                    screen.stop_mode()
                    pump.setPercent('a', 0, '+')
                    pump.setPercent('b', 0, '+')
                    co_calculated = 0
                    co_calculated_raw = 0
                    status = 3
                else:
                    if inSafe is False:  #NO PURGE RUN MODE
                        status = 0
                        co_calculated = int(co)
                        co_calculated_raw = int(ec_co_sensor.raw_ppm)
                        pump.setPercent('a', 0, '+')
                        pump.setPercent('b', pump_saved, '+')
                        screen.run_mode()
                    elif inSafe is True:   #NO PURGE SAFE MODE
                        status = 1
                        co_calculated = int(co)*5
                        co_calculated_raw = int(ec_co_sensor.raw_ppm)*5
                        screen.safe_mode()
                    cmd_handler()
            except Exception as e:
                err = "Normal run mode failure: " + str(e)
                print(err)
                screen.error(err)
        cmd_handler()
        lastCO = co
        try:
            screen.send_admin(round(current, 2), round(voltage, 2), int(ec_co_sensor.raw_ppm), int(ec_no_sensor.raw_ppm), round(o2_sensor.o2, 1), int(inline_humidity), round(inline_temp, 1), round((o2_sensor.pressure*0.401865), 1))
        except Exception as e:
            err = "Send Admin Values Failure: " + str(e)
            print(err)
            screen.error(err)
        #==================================================== XBee Stuff ============================================================
        try:
            cmd_handler()
            if locationSend is True:
                try:
                    if network is True:
                        xbee.send("L", location[0], location[1], location[2], location[3], location[4], location[5])
                        screen.notify("Location Sent!", "")
                    else:
                        screen.notify("Location Not Sent!", "Network Disconnected!\r\nTry again.")
                except Exception as e:
                    screen.notify("Location Not Sent!", "Try again!")
                    err = "Xbee Send Location Error (resending): " + str(e)
                    print(err)
                    screen.error(err)
                locationSend = False
                time.sleep(0.005)
            cmd_handler()
            if healthSend is True:
                try:
                    if network is True:
                        #xbee.send("HR", health[0], health[1], health[2], health[3], health[4], health[5])
                        screen.notify("Health Report Sent!", "")
                    else:
                        screen.notify("Health Report Not Sent!", "Network Disconnected!\r\nTry again.")
                except Exception as e:
                    screen.notify("Health Report Not Sent!", "Try again!")
                    err = "Xbee Send Health Report Error (resending): " + str(e)
                    print(err)
                    screen.error(err)
                healthSend = False
                time.sleep(0.005)
            cmd_handler()
            if saveSend is True:
                try:
                    if network is True:
                        if save[0] == "pressure":
                            xbee.send("S", save[0], save[1], save[2])
                        elif save[0] == "flue":
                            xbee.send("S", save[0], save[1], save[2], save[3])
                        screen.notify("Save Prs/Flue Sent!", "")
                    else:
                        screen.notify("Save Prs/Flue Not Sent!", "Network Disconnected!\r\nTry again.")
                except Exception as e:
                    screen.notify("Save Prs/Flue Not Sent!", "Try again!")
                    err = "Xbee Send Save Data Error (resending): " + str(e)
                    print(err)
                    screen.error(err)
                saveSend = False
                time.sleep(0.005)
            cmd_handler()
            if time.ticks_diff(time.ticks_ms(), last_health) > 60000: ## 60 seconds
                last_health = time.ticks_ms()
                try:
                    xbee.send("H1", round(voltage, 2), inline_temp, inline_temp)
                    time.sleep(0.005)
                except Exception as e:
                    err = "Xbee Send H1 Error: " + str(e)
                    print(err)
                    screen.error(err)
                cmd_handler()
                try:
                    xbee.send("ECH", co, ec_co_sensor.raw_ppm, 0)
                    time.sleep(0.005)
                except Exception as e:
                    err = "Xbee Send EC Health Error: " + str(e)
                    print(err)
                    screen.error(err)
                cmd_handler()
                try:
                    if inPurge is True or inSafe is True:
                        xbee.send("H2", o2_sensor.o2, o2_sensor.ppo2, o2_sensor.temp, o2_sensor.pressure, o2_sensor.errors) 
                        time.sleep(0.005)
                    else:
                        xbee.send("H2", o2_sensor.o2, o2_sensor.ppo2, o2_sensor.temp, o2_sensor.pressure, o2_sensor.errors) 
                        time.sleep(0.005)
                except Exception as e:
                    err = "Xbee Send H2 Error: " + str(e)
                    print(err)
                    screen.error(err)
            cmd_handler()
            if time.ticks_diff(time.ticks_ms(), last_normal) > 1000: ## 1 seconds
                last_normal = time.ticks_ms()

                cmd_handler()
                try:
                    o2_sensor.get()
                    newo2 = (((o2_sensor.ppo2/o2_sensor.pressure)*100)-o2offset)*o2gain
                    oxy.add_point(newo2)
                    cmd_handler()
                except Exception as e:
                    err = "O2 Read Error: " + str(e)
                    print(err)
                    screen.error(err)

                if inStop is False:
                    try:
                        xbee.send("C", oxy.average(), co_calculated, p_inh2o)
                        time.sleep(0.005)
                        while r == "":
                            r = xbee.DEVICE.readline()
                        if r == b'V3-1\r\n':
                            screen.set_signal_strength(2)
                            network = True
                        elif r == b'V3-0\r\n':
                            screen.set_signal_strength(0)
                            network = False
                        r = ""
                        cmd_handler()
                    except Exception as e:
                        err = "Xbee Send Normal Values Error: " + str(e)
                        print(err)
                        screen.error(err)
                    cmd_handler()
                    try:
                        xbee.send("EC", co_calculated, no)
                        time.sleep(0.005)
                        cmd_handler()
                    except Exception as e:
                        err = "Xbee Send EC Data Error: " + str(e)
                        print(err)
                        screen.error(err)
                    cmd_handler()

                elif inStop is True:
                    try:
                        xbee.send("CM", p_inh2o)
                        time.sleep(0.005)
                        while r == "":
                            r = xbee.DEVICE.readline()
                        if r == b'V3-1\r\n':
                            screen.set_signal_strength(2)
                            network = True
                        elif r == b'V3-0\r\n':
                            screen.set_signal_strength(0)
                            network = False
                        r = ""
                    except Exception as e:
                        err = "Xbee Send 'CM' Error: " + str(e)
                        print(err)
                        screen.error(err)
            slope1 += 1
            cmd_handler()
        except Exception as e:
            err = "XBee Thread Failure: " + str(e)
            print(err)
            screen.error(err)
    except Exception as e:
        err = "Main Loop Runtime Error: " + str(e)
        print(err)
        screen.error(err)
