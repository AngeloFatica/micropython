# DEVICE - Nextion Display Enhanced Series
# VENDOR - https://nextion.tech/
# DATASHEET - https://nextion.tech/enhanced-series-introduction/
from Formats import fmt, err
import utime

class Screen:
    
    def __init__(self, device, debug_mode=False):
        self.write_block = False
        self.device = device
        self.debug_mode = debug_mode
    
    def __error_check(self):
        pass
        #rtn_msg = self.device.readline()
        #if rtn_msg is not None:
        #    code = str(rtn_msg).split('\\')[1]
        #    if code != "x01" and code != "xfe":
        #        raise Exception(err[code])
        #    else:
        #        return code

    def set_write_block(self):
        self.write_block = True
        
    def reset_write_block(self):
        self.write_block = False

    def write(self, f, **kwargs):
        if self.write_block:
            return "block"
        try:
            msg = f.format(**kwargs)
            if self.debug_mode is True:
                print(msg)
            written = self.device.write(msg)
            if written is None or written == 0:
                raise Exception("Didn't write")
            else:
                return True
        except Exception as e:
            return e
    
    def change_page(self, page: str):
        self.write(fmt["PAGE"], page=page )
        self.__error_check()

    def refresh_elem(self, elem_name: str):
        self.write(fmt["REFRESH"], elem_name=elem_name)
        self.__error_check()    

    def refresh_page(self):
        self.write(fmt["REFRESH"], elem_name="0")
        self.__error_check()

    def write_global_text(self, page: str, elem_name: str, data ):
        self.write(fmt["GTEXT"], page=page, elem_name=elem_name, data=str(data) )
        self.__error_check()
        
    def write_text(self, elem_name: str, data ):
        self.write(fmt["TEXT"], elem_name=elem_name, data=str(data) )
        self.__error_check()

    def change_text_color(self, page: str, elem: str, color ):
        self.write(fmt["TEXT_COLOR"], page=page, elem_name=elem, color=str(color) )
        self.__error_check()

    def write_global(self, name: str, value ):
        self.write(fmt["GLOBAL"], name=name, value=value )
        self.__error_check()

    def touch_component(self, elem: str ):
        self.write(fmt["TOUCH"], elem_name=elem, state=1 )
        self.__error_check()
    
    def release_component(self, elem: str ):
        self.write(fmt["TOUCH"], elem_name=elem, state=0 )
        self.__error_check()

    def set_bar_value(self, page: str, elem: str, value ):
        self.write(fmt["BAR"], page=page, elem_name=elem, data=value )
        self.__error_check()

    def send_to_graph(self, ID: int, channel: int, value: int):
        self.write(fmt["GRAPH"], elem=ID, ch=channel, value=value )
        #if self.__error_check() == "xfe":
        #    self.write(hex(value))
        self.__error_check()
        
    def set_visibility(self, elem: str, vis: bool):
        self.write(fmt["VIS"], elem_name=elem, data=int(vis))

    def get(self, elem: str):
        self.write(fmt["GET"], elem_name=elem)
        ret = self.device.readline()
        return ret

    def read_instr(self):
        instr = self.device.readline()
        if instr is not None:
            instr = str(instr)[2:-1]
            instr = instr.split()
            # print(instr)
            if instr[0] == "scr":
                return instr[1:]
            else:
                return None
            
class PSNERGY_Screen(Screen):
    gFlue     = 13
    gPR       = 12
    grPRScale = 5
    grO2Max   = 21
    notifs    = True
    
    def __init__(self, device, debug_mode=False):
        super().__init__(device, debug_mode)
    
    def send_O2(self, value: float):
        self.write_global_text("home", "tO2", value )
        value = (value/self.grO2Max)*100
        print(value)
        self.send_to_graph(self.gFlue, 0, int(value))
    
    def send_CO(self, value: int):
        self.write_global_text("home", "tCO", int(value) )

    def send_NO(self, value: int):
        self.write_global_text("home", "tNOX", int(value) )
        
    def send_PR(self, RT: float, MA: float):
        self.write_global_text("home", "tPR", round(RT,2) )
        self.write_global_text("manometer", "tRT", round(RT,2) )
        self.write_global_text("manometer", "tMA", round(MA,2))
        RT = self.grPRScale*RT + 50
        MA = self.grPRScale*MA + 50
        self.send_to_graph(self.gFlue, 1, int(RT))
        self.send_to_graph(self.gPR, 0, int(RT))
        self.send_to_graph(self.gPR, 1, int(MA))
        
    def boot_progress(self, message: str, percent: int):
        self.set_bar_value("boot","jBoot",percent)
        self.write_text("tBoot",message)
        
    def set_battery_percent(self, percent: int):
        self.write_global("PERCENT",percent)
        self.touch_component("mBar")
        
    def set_signal_strength(self, strength: int):
        self.write_global("SIGNAL",strength)
        self.touch_component("mBar")
        
    def set_desiccant(self, good: bool):
        self.write_global("HUMID", int(good))
        self.touch_component("mBar")
    
    def set_device_name(self, name: str):
        self.write_global_text("boot","analyzerN", name)
        self.touch_component("mBar")
        
    def error(self, err: str):
        self.write_global("ERROR",1)
        self.touch_component("mBar")
        self.device.write(b"errLog.slErrors.txt+=\"{error}\x0d\x0a\"\xff\xff\xff".format(error=err))
        
    def notify(self, title: str, msg: str):
        if self.notifs:
            self.write_global_text("notification", "tTitle", title)
            self.write_global_text("notification", "tMessage", msg)
            self.change_page("notification")
        
    def notifications(self, notif: bool):
        self.notifs = notif
        
    def magic_number(self, num: int):
        self.write_global_text("home", "tMagic", num)
        self.write_global("magic", 1)
        self.set_visibility("home.tMagic", True)
    
    def stop_mode(self):
        self.write_global("state",0)
        self.touch_component("mStop")
        self.write(fmt["ENABLE"], elem_name="bState")
    
    def run_mode(self):
        self.write_global("state",1)
        self.touch_component("mRun")
        self.write(fmt["ENABLE"], elem_name="bState")
    
    def safe_mode(self):
        self.write_global("state",2)
        self.touch_component("mSafe")
        self.write(fmt["ENABLE"], elem_name="bState")
        
    def purge_mode(self):
        self.write_global("state",3)
        self.touch_component("mPurge")
        self.write(fmt["DISABL"], elem_name="bState")
        
    def protect_mode(self):
        #shit is fucked protect everything
        self.write_global("state",4)
        self.touch_component("mProtect")
        self.write(fmt["DISABL"], elem_name="bState")
        
    def populate_calvals(self, O2High, O2Low, COHigh, COLow, NOHigh, NOLow, PROffset ):
        self.write_text("tO2h", str(O2High))
        self.write_text("tO2l", str(O2Low))
        self.write_text("tCOh", str(COHigh))
        self.write_text("tCOl", str(COLow))
        self.write_text("tNOh", str(NOHigh))
        self.write_text("tNOl", str(NOLow))
        self.write_text("tPRo", str(PROffset))
    
    def cal_O2(self, O2_Raw: int):
        self.write_text("tO2", O2_Raw)
        gval = (O2_Raw/22) * 150
        self.send_to_graph(6,0, int(gval))
        
    def cal_CO(self, CO_Raw: int):
        self.write_text("tCO", CO_Raw)
        gval = (CO_Raw/500) * 150
        self.send_to_graph(6,0, int(gval))
    
    def cal_NO(self, NO_Raw: int):
        self.write_text("tNO", NO_Raw)
        gval = (NO_Raw/500) * 150
        self.send_to_graph(6,0, int(gval))

    def send_shutdown(self, msg: str, msg2: str, val: int):
        self.write_global_text("shutdown","tMessage",msg)
        self.write_global_text("shutdown","tVal",val)
        self.write_global_text("shutdown","tMessage2",msg2)
