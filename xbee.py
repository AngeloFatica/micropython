# C - core
    # A01 20.70 40 1.0 psi 9
# H - health
    # A01 27.6 28.2 3.72 -78
# A - alert
# L - log
# S - save
# U - unclassified

DEVICE = None
ID = None
WRITE_BLOCK = False

def send(kind, *args):
    global WRITE_BLOCK
    if WRITE_BLOCK is False:
        global ID
        global DEVICE
        body = list(args)
        #body.insert(0, ID)
        body.insert(0, kind)
        msg = ' '.join((str(i) for i in body))
        msg = '{} {}\n'.format(msg, len(msg))
        DEVICE.write(bytes(msg, 'utf-8'))

def checksum(values):
    try:
        reported = int(values.pop(-1)) #remove the sum
        calculated = sum(len(i) for i in values) #add up everything in the list
        calculated = calculated + len(values) - 1 #add the length of the spaces back in
        if reported == calculated:
            return(1)
        else:
            print('XBEE Checksum error: Reported: {} Calculated: {} {}'.format(reported, calculated, values))
            return(-1)
    except ValueError as e:
        print("XBEE Checksum error:", e, values)
        return(-1)
    except IndexError as e:
        print("XBEE Error", e, values)
        return(-1)

def check_input():
    try:
        global DEVICE
        while DEVICE.in_waiting > 0:
            rx = DEVICE.readline()
            if rx is not None:
                msg = rx.decode('utf-8').rsplit()
                print('\t\t\t\t Msg from xbee', rx)
                if checksum(msg) != -1:
                    if msg[0] != "xbee":
                        continue
                    else:
                        if msg[1] == "setting":
                            global WRITE_BLOCK
                            WRITE_BLOCK = True
                        if msg[1] == "CLEAR":
                            global WRITE_BLOCK
                            WRITE_BLOCK = False
                        else:
                            return(msg)
    except Exception as e:
        print("XBEE read error", e)