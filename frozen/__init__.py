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
#         rtn_msg = self.device.readline()
#         if rtn_msg is not None:
#             code = str(rtn_msg).split('\xff\xff\xff')[0]
#             if code != "x01" and code != "xfe" and code != "x12":
#                 raise Exception(err[code])
#             else:
#                 return code

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
        instr = self.device.read()
        if instr is not None:
            instr = str(instr)[2:-1]
            instr = instr.split("\\xff\\xff\\xff")
            for cmd in instr:
                cmd = cmd.split()
#                 print(cmd)
                if cmd[0] == "scr":
                    return cmd[1:]
                elif cmd[0] == "\\x12":
                    return None
                else:
#                     print(cmd[0][1:])
                    # raise Exception(err[cmd[0][1:]])
                    return None
            
class PSNERGY_Manometer(Screen):
    gPR       = 1
    grPRScale = 30
    notifs    = False
    
    def __init__(self, device, debug_mode=False):
        super().__init__(device, debug_mode)
        
    def send_PR(self, RT: float, MA: float):
        self.write_global_text("manometer", "tRT", round(RT,2) )
        self.write_global_text("manometer", "tMA", round(MA,2))
        MA = 66/self.grPRScale*MA + 66
        RT = 66/self.grPRScale*RT + 66
        self.send_to_graph(self.gPR, 0, int(RT))
        self.send_to_graph(self.gPR, 1, int(MA))

    def send_admin(self, RT: float, MA: float, offset: float, temp: float, voltage: float, current: float):
        self.write_global_text("admin", "tART", RT)
        self.write_global_text("admin", "tAMA", MA)
        self.write_global_text("admin", "tAOFF", offset)
        self.write_global_text("admin", "tATEMP", temp)
        self.write_global_text("admin", "tAVOLT", voltage)
        self.write_global_text("admin", "tACURR", current)

    def magic_number(self, num: int):
        self.write_global_text("manometer", "tMagic", num)
        self.write_global("magic", 1)
        self.set_visibility("tMagic", 1)
        
    def boot_progress(self, message: str, percent: int):
        self.set_bar_value("boot","jBoot",percent)
        self.write_text("tBoot",message)
        
    def set_battery_percent(self, percent: int):
        self.write_global("PERCENT",percent)
        self.touch_component("devid")
        
    def set_signal_strength(self, strength: int):
        self.write_global("SIGNAL",int(strength))
        self.touch_component("devid")
    
    def set_device_name(self, name: str):
        self.write_global_text("boot","manoN", name)
        self.touch_component("devid")
        
    def error(self, err: str):
        self.write_global("ERROR",1)
        self.touch_component("devid")
        self.device.write(b"errLog.tErrors.txt+=\"{error}\x0d\x0a\"\xff\xff\xff".format(error=err))
