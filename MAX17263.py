from machine import I2C

class MAX17263:
	# Voltages
	VCELL 		= 	0x09
	AvgVCell 	= 	0x19
	VRipple 	= 	0xbc

	# Temperatures
	Temp		=	0x08
	AvgTA		=	0x16
	AIN			=	0x27
	TGain		=	0x2c
	TOff		=	0x2d
	DieTemp		=	0x34

	# Capacities
	RepCap		=	0x05
	QResidual	=	0x0c
	MixCap		=	0x0f
	FullCapRep	=	0x10
	DesignCap	=	0x18
	AvCap		=	0x1f
	FullCapNom	=	0x23
	dQAcc		=	0x45
	VFRemCap	=	0x4a
	QH			=	0x4d
	AtQResidual	=	0xdc
	AtAvCap		=	0xdf

	# Currents
	AtRate		=	0x04
	Current		=	0x0a
	AvgCurrent	=	0x0b
	COff		=	0x2f

	# Timers
	TTE			=	0x11
	TTF			=	0x20
	Timer		=	0x3e
	TimerH		=	0xbe
	AtTTE		=	0xdd

	# Percentages
	RepSOC		=	0x06
	Age			=	0x07
	MixSOC		=	0x0d
	AvSOC		=	0x0e
	dp_acc		=	0x46
	AtAvSOC		=	0xde

	# Special Regs
	QRTable00	=	0x12
	FullSocThr	=	0x13
	Config		=	0x1d
	IChgTerm	=	0x1e
	QRTable10	=	0x22
	LearnCfg	=	0x28
	FilterCfg	=	0x29
	RelaxCfg	=	0x2a
	MiscCfg		=	0x2b
	QRTable20 	=	0x32
	FullCap 	=	0x35
	RComp0		=	0x38
	VEmpty		=	0x3a
	FStat		= 	0x3d
	QRTable30	=	0x42

	def __init__(self, ADDR: hex, BUS, sense_resistor=0.01):
		self.ADDR=ADDR
		self.BUS=BUS
		self.sense=sense_resistor

	def read_percentage(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")/256
		return ret

	def read_voltage(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")*0.000078125
		return ret

	def read_current(self, reg: int):
		b = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		val = (not(b[1]&0x80)>>7)*int.from_bytes(b,"little")&0x7fff | ((b[1]&0x80)>>7)*(-1*((((b[1]^0x00ff)<<8)|(b[1]^0x00ff))+1))
		# print("val: ", val)
		ret = val*(0.0000015625/self.sense)
		# print("after: ",ret)
		return ret

	def read_capacity(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")*(0.000005/self.sense)
		return ret

	def read_temperature(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")/256
		return ret
		# (not(b[0]&0x80)>>7)*int.from_bytes(b,"big")&0x7fff | ((b[0]&0x80)>>7)*(-1*((((b[0]^0x00ff)<<8)|(b[1]^0x00ff))+1))

	def read_time(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")*5.625
		return ret

	def read_resistance(self, reg: int):
		val = self.BUS.readfrom_mem(self.ADDR, reg, 2)
		ret = int.from_bytes(val, "little")/4096
		return ret

	def batt_percent(self):
		return self.read_percentage(self.RepSOC)

	def current_draw(self):
		return self.read_current(self.Current)

	def oc_voltage(self):
		return self.read_voltage(self.AvgVCell)

	def batt_temp(self):
		return self.read_voltage(self.Temp)