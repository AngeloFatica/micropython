import time
import binascii
import machine
import pyb

def __calcCRC( datagram: bytearray) -> int:
    crc = 0xFF
    for byte in datagram:
        crc ^= byte
        for i in range(0,8):
            if((crc & 0x80) != 0):
                crc = ((crc << 1) ^ 0x9B) & 0xFF
            else:
                crc <<= 1
    return crc

class frame():

	def __init__(self, num, baud):
		self.xbee = machine.UART(num, baud)


	def checksum(self, x):
		added = 0
	
		for byte in x:
			added += byte
	
		final = (255 - added)
		final = final & 0xff
		return hex(final)
	
	def make_frame(self, x):
		check = 0
		length = 0
		x = str(x)
		
		temp = binascii.unhexlify(x)

		length = (len(temp))
		lengthnew = '%04x' % length
		lengthnew = str(lengthnew)
		check = self.checksum(temp)
		checknew = '%02x' % int(check)
		checknew = str(checknew)
	
		y = "{}{}{}{}0D".format("7e",lengthnew,x,checknew)
		y = y.replace('0x','')

		frame = binascii.unhexlify(y)
	
		return frame
	
	
	def generate_frame(self, cmd_type, cmd_ID, cmd_mac=None, r_w="r", cmd_data=None, AT_param=None):
	
		if cmd_type == "AT":
			cmd_type = "08"
			temp = str(binascii.hexlify(AT_param))
			temp = temp.replace("b'", '')
			temp = temp.replace("'", '')
			cmd_param = temp
			if r_w == "r":
				a = "{}{}{}".format(cmd_type, cmd_ID, cmd_param)
			elif r_w == "w":
				temp = str(binascii.hexlify(cmd_data))
				temp = temp.replace("b'", '')
				temp = temp.replace("'", '')
				cmd_data = temp
				a = "{}{}{}{}".format(cmd_type, cmd_ID, cmd_param, cmd_data)
	
		elif cmd_type == "TX":
			cmd_type = "10"
			cmd_data = str(binascii.hexlify(cmd_data))
			cmd_data = cmd_data.replace("b'", '')
			cmd_data = cmd_data.replace("'", '')
			a = "{}{}{}{}{}".format(cmd_type, cmd_ID, cmd_mac, "FFFE0000", cmd_data)
	
		elif cmd_type == "ATR":
			cmd_type = "17"
			cmd_param = str(binascii.hexlify(AT_param))
			cmd_param = cmd_param.replace("b'", '')
			cmd_param = cmd_param.replace("'", '')
			if r_w == "r":
				a = "{}{}{}{}{}".format(cmd_type, cmd_ID, cmd_mac, "FFFE02", cmd_param)
			elif r_w == "w":
				cmd_data = str(binascii.hexlify(cmd_data))
				cmd_data = cmd_data.replace("b'", '')
				cmd_data = cmd_data.replace("'", '')
				a = "{}{}{}{}{}{}".format(cmd_type, cmd_ID, cmd_mac, "FFFE02", cmd_param, cmd_data)
		

		return (self.make_frame(a))
	
	
	def receiveFrame(self, x):
		temp = x
		print(x)

		length = x[1:3]
		cmd_type = x[3:4]
		cmd_ID = x[4:5]
		check = x[-1]
	
		if cmd_type == b'\x10': #Tx Request
			cmd_type = "10-TxRequest"
			cmd_mac = x[5:13]
			backup_data = x[13:17]
			cmd_data = x[17:-1]
	
			a = "Delimeter: {} \nCommand Length: {} \nCommand Type: {} \nCommand ID: {} \nCommand MAC: {} \nBackup Data: {} \nCommand Data: {} \nChecksum: {}".format("7E", length, cmd_type, cmd_ID, cmd_mac, backup_data, cmd_data, check)
	
		elif cmd_type == b'\x97': #Remote AT command
			cmd_type = "97-ATCMD Response From Remote XBee"
			cmd_mac = x[5:13]
			cmd_16bit = x[13:15]
			cmd_param = x[15:17]
			cmd_status = x[17:18]
			cmd_data = x[18:-1]
	
			a = "Delimeter: {} \nCommand Length: {} \nCommand Type: {} \nCommand ID: {} \nCommand MAC: {} \nCommand 16-bit Address: {} \nCommand Parameter: {} \nCommand Status: {} \nCommand Data: {} \nChecksum: {}".format("7E", length, cmd_type, cmd_ID, cmd_mac, cmd_16bit,cmd_param, cmd_status, cmd_data, check)
	
		elif cmd_type == b'\x88': #AT command
			cmd_type = "88-ATCMD Response From XBee"
			cmd_param = x[5:7]
			cmd_status = x[7:8]
			cmd_data = x[8:-1]
			#if cmd_param != b'BI' and cmd_param != b'NI' and cmd_param != b'ID':
			#	cmd_data = int.from_bytes(cmd_data, "big")
	
			a = "Delimeter: {} \nCommand Length: {} \nCommand Type: {} \nCommand ID: {} \nCommand Parameter: {} \nCommand Status: {} \nCommand Data: {} \nChecksum: {}".format("7E", length, cmd_type, cmd_ID, cmd_param, cmd_status, cmd_data, check)

		elif cmd_type == b'\x8b':
			cmd_type = "8B-Transmit Status"
			cmd_mac = x[5:7]
			tx_retries = x[7:8]
			cmd_status = x[8:9]
			discovery_status = x[9:10]

			cmd_data = {"Type":cmd_type,
						"Destination":cmd_mac,
						"Retries":tx_retries,
						"Status":hex(int.from_bytes(cmd_status, "big")),
						"Discovery":discovery_status
						}
		else:
			cmd_data = "Receive Frame Error: cmd type not recognized!"

		#print(a)
	
		return cmd_data
	
	def writeChanges(self):
		try:
			w = self.generate_frame(cmd_type="AT", cmd_ID="10", AT_param="WR")
			self.xbee.write(w)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			return 1
		except Exception as e:
			print("Write Changes Failure: ", e)
			return -1
	
	def readParam(self, param):
		try:
			r = None
			p = self.generate_frame(cmd_type="AT", cmd_ID="08", r_w="r", AT_param=str(param))
			self.xbee.write(p)
			start = time.ticks_ms()
			while r is None and time.ticks_diff(time.ticks_ms(), start) < 5000:
				r = self.xbee.read()
				self.xbee.write(p)
			if r is None:
				return "Read Param Error: No Data Received"
			else:
				return self.receiveFrame(r)
		except Exception as e:
			print("Read NI Error: ", e)
			return -99

	def writeParam(self, param, value):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="02", cmd_data=str(value), AT_param=param, r_w="w") #Change Node Identifier to "New Name"
		
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
		
			self.writeChanges()
	
		except Exception as e:
			print("Write Parameter Error: ", e)
			return -1

	def readAPI(self):
		try:
			r = None
			p = self.generate_frame(cmd_type="AT", cmd_ID="08", r_w="r", AT_param="AP")
			self.xbee.write(p)
			start = time.ticks_ms()
			while r is None and time.ticks_diff(time.ticks_ms(), start) < 5000:
				r = self.xbee.read(10)
				self.xbee.write(p)
			if r is None:
				return "Read Param Error: No Data Received"
			else:
				return self.receiveFrame(r)
		except Exception as e:
			print("Read NI Error: ", e)
			return -99
	
	
	def readNI(self):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="08", r_w="r", AT_param="NI")
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			return self.receiveFrame(r)
		except Exception as e:
			print("Read NI Error: ", e)
			return -1
	
	def readDB(self):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="04", r_w="r", AT_param="DB")
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			return self.receiveFrame(r)
		except Exception as e:
			print("Read DB Error: ", e)
			return -1
	
	def readDBRemote(self, mac):
		try:
			p = self.generate_frame(cmd_type="ATR", cmd_ID="01", cmd_mac=str(mac), AT_param="DB", r_w="r")
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			return self.receiveFrame(r)
		except Exception as e:
			print("Read DB Error: ", e)
			return -1
	
	def writeNI(self, nodeID):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="02", cmd_data=str(nodeID), AT_param="NI", r_w="w") #Change Node Identifier to "New Name"
		
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
		
			self.writeChanges()
	
		except Exception as e:
			print("Write NI Error: ", e)
			return -1
	
	def readID(self):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="09", r_w="r", AT_param="ID")
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			print(r)
			return self.receiveFrame(r)
		except Exception as e:
			print("Read ID Error: ", e)
			return -1
	
	def writeID(self, ID):
		try:
			p = self.generate_frame(cmd_type="AT", cmd_ID="06", r_w="w", cmd_data=ID, AT_param="ID") #Change Node Identifier to "New Name"
		
			self.xbee.write(p)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
		
			w = self.generate_frame(cmd_type="AT", cmd_ID="03", AT_param="WR")   #Write changes to non-volatile memory
		
			self.xbee.write(w)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
		
			return 1
		except Exception as e:
			print("Write NI Error: ", e)
			return -1
	
	def txData(self, mac, data):
		try:
			msg = '{} {}\n'.format(data, chr(__calcCRC(bytes(data, 'utf-8'))))
			p = self.generate_frame(cmd_type="TX", cmd_ID="04", cmd_mac=str(mac), cmd_data=str(msg))
			self.xbee.write(p)
		except Exception as e:
			print("TX Data Error: ", e)
			return -1

	def xbRead(self, x=None):
		try:
			count = 0
			r=None
			while count < 5 and r == None:
				if x == None:
					r = self.xbee.read()
				else:
					r = self.xbee.read(x)
				count+=1
			if r != None:
				return r
			else:
				return -1
		except Exception as e:
			print(e)

	def xbWrite(self, x):
		try:
			self.xbee.write(x)
			while True:
				r = self.xbee.read()
				if r is not None:
					break
			return r
		except Exception as e:
			print(e)

	def transInit(self, nodeID, blueID, devRole='0', panID='ABCD', watchDog='50', coordVerify='1', baud='7', apiMode='1', bluetooth='1', scanChannels="FFFF"):
		time.sleep(2)
		self.xbee.write(b'+++')
		time.sleep(0.5)
		status=None
		while status is None:
			status = self.xbee.read()
		if 'OK' in status:
			print("Entered Command Mode...")
		else:
			print("Could not enter command mode!")
			return -1

		status = None
		self.xbee.write('ATCE {}\r\n'.format(str(devRole)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed device role to: ", devRole)
		else:
			print("Could not change device role!")
			return -1

		status = None
		self.xbee.write('ATID {}\r\n'.format(str(panID)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed PAN ID to: ", panID)
		else:
			print("Could not change PAN ID!")
			return -1

		status = None
		self.xbee.write('ATNW {}\r\n'.format(str(watchDog)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed watch dog to: ", watchDog)
		else:
			print("Could not change watch dog!")
			return -1


		status = None
		self.xbee.write('ATJV {}\r\n'.format(str(coordVerify)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed coordinator verification to: ", coordVerify)
		else:
			print("Could not change coordinator verification!")
			return -1


		status = None
		self.xbee.write('ATNI {}\r\n'.format(str(nodeID)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed Node ID to: ", nodeID)
		else:
			print("Could not change Node ID!")
			return -1

		status = None
		self.xbee.write('ATBD {}\r\n'.format(str(baud)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed Baud to: ", baud)
		else:
			print("Could not change Baud!")
			return -1

		status = None
		self.xbee.write('ATAP {}\r\n'.format(str(apiMode)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed API Mode to: ", apiMode)
		else:
			print("Could not change API Mode!")
			return -1


		status = None
		self.xbee.write('ATBT {}\r\n'.format(str(bluetooth)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed bluetooth to: ", bluetooth)
		else:
			print("Could not change bluetooth!")
			return -1

		status = None
		self.xbee.write('ATBI {}\r\n'.format(str(blueID)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed bluetooth ID to: ", blueID)
		else:
			print("Could not change bluetooth ID!")
			return -1

		status = None
		self.xbee.write('ATSC {}\r\n'.format(str(scanChannels)))
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changed Scan channels to: ", scanChannels)
		else:
			print("Could not change scan channels!")
			return -1

		status = None
		self.xbee.write('ATWR\r\n')
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Changes Successfully written to device!")
		else:
			print("Changes could not be written to device!")
			return -1

		status = None
		self.xbee.write('ATCN\r\n')
		while status is None:
			status = self.xbee.read()
		
		if 'OK' in status:
			print("Exited Command Mode!")
		else:
			print("Could not exit command mode!")
			return -1

		return 1







