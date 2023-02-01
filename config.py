class Config:
    device_id = ""      # Read from FRAM on conf load
    vmax_meas = 0
    vmin_meas = 0
    gas_offset    = 0
    air_offset    = 0
    tx_int        = 2000 # ms
    prs_max = 0
    prs_min = 0
    is_set = False      # Set this to true if loaded from FRAM or RXed from Xbee
    
    def __init__(self):
        try:
            self.load()
        except Exception as e:
            print("Cannot do this", e)

    def sync(self): #writes all local variables to text file
        f = open('ConfigValues.txt', 'w')
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()

    def print_text_file(self): #prints the variables in the text file
        count = 0
        length = 9
        print("\n+-------Config values from txt file------+")
        with open('ConfigValues.txt') as f:
            while count < length:
                print(f.readline())
                count+=1
            f.close()
        print("+----------------------------------------+")

    def set_device_id(self, device_id):
        f = open('ConfigValues.txt', 'w')
        self.device_id = device_id
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_device_id(self):
        return self.device_id

    def set_vmin_meas(self, vmin_meas):
        f = open('ConfigValues.txt', 'w')
        self.vmin_meas = vmin_meas
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_vmin_meas(self):
        return self.vmin_meas

    def set_vmax_meas(self, vmax_meas):
        f = open('ConfigValues.txt', 'w')
        self.vmax_meas = vmax_meas
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_vmax_meas(self):
        return self.vmax_meas

    def set_gas_offset(self, gas_offset):
        f = open('ConfigValues.txt', 'w')
        self.gas_offset = gas_offset
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_gas_offset(self):
        return self.gas_offset

    def set_air_offset(self, air_offset):
        f = open('ConfigValues.txt', 'w')
        self.air_offset = air_offset
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_air_offset(self):
        return self.air_offset

    def set_tx_int(self, tx_int):
        f = open('ConfigValues.txt', 'w')
        self.tx_int = tx_int
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_tx_int(self):
        return self.tx_int

    def set_prs_max(self, prs_max):
        f = open('ConfigValues.txt', 'w')
        self.prs_max = prs_max
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_prs_max(self):
        return self.prs_max

    def set_prs_min(self, prs_min):
        f = open('ConfigValues.txt', 'w')
        self.prs_min = prs_min
        f.write(str(self.device_id)+'\n')
        f.write(str(self.vmax_meas)+'\n')
        f.write(str(self.vmin_meas)+'\n')
        f.write(str(self.gas_offset)+'\n')
        f.write(str(self.air_offset)+'\n')
        f.write(str(self.tx_int)+'\n')
        f.write(str(self.prs_max)+'\n')
        f.write(str(self.prs_min)+'\n')
        f.write(str(self.is_set)+'\n')
        f.close()
        return
    def get_prs_min(self):
        return self.prs_min
    
    def load(self):  #load values from config text into local variables
        config = list(range(8))
        iterator = 0
        chars = 0
        name = ""

        with open('ConfigValues.txt') as f: 
            dummy = f.readline()
            for chars in range(len(dummy)):  #Gets rid of '\n' in device id string, loads id into name
                if dummy[chars] != '\n':
                    name += str(dummy[chars])
                else:
                    break
            for iterator in range(len(config)): #Loads the rest of the number values into config array
                config[iterator] = f.readline()
                iterator+=1
        f.close()
        
        self.device_id = name   #Sets all local variables from text file values
        self.vmax_meas = float(config[0])
        self.vmin_meas = float(config[1])
        self.gas_offset = float(config[2])
        self.air_offset = float(config[3])
        self.tx_int = float(config[4])
        self.prs_max = float(config[5])
        self.prs_min = float(config[6])
        self.is_set = True
        
        return "+----- Values loaded into local variables from text -----+"
