'''
    This is a standard library for Trinamic single wire UART
	write_reg() - used to write to a specific register on the tmc chip. 
	Note: no need to add the write bit to the address
	read_reg() - used to read a specific register from the tmc chip. Returns the 4 data bytes as a byte array
'''

import os
import utime
import machine
from machine import Pin

SYNCBYTE = 0x05

#registers
GCONF = 0x00
GSTAT = 0x01
IFCNT = 0x02
SLAVECONF = 0x03
IOIN = 0x06
CURRENT_LIMIT = 0x10
PWM_AB = 0x22
CHOPCONF = 0x6C
DRV_STATUS = 0x6F
PWNCONF = 0x70

class TMC_UART:
    MOTOR_A_SPEED = [0x00, 0x00]
    MOTOR_B_SPEED = [0x00, 0x00]

    def __init__(self, tmcUART, slave_address):
        print("Starting UART with the following parameters")
        self.uart = tmcUART
        self.slave_address = slave_address
        print(self.uart)

    def __reverse_Bits(self, n):
        result = 0
        for i in range(8):
            result <<= 1
            result |= n & 1
            n >>= 1
        return result

    def __calcCRC(self, datagram):
        crc = 0x00
        for byte in datagram:
            byte = self.__reverse_Bits(byte)
            crc ^= byte
            for i in range(0,8):
                if((crc & 0x80) != 0):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc <<= 1
        return crc
    
    # Will return a integer
    def __convert(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def write_reg( self, reg_address, data ):
    	reg_address	= reg_address ^ 0x80
        writeDatagram = [SYNCBYTE,self.slave_address,reg_address]+ data +[0]
        writeDatagram[7] = self.__calcCRC(writeDatagram[:7])
        self.uart.write(bytearray(writeDatagram))
        self.uart.readline()

    def read_reg( self, reg_address ):
        readDatagram = [SYNCBYTE,self.slave_address,reg_address,0x00]
        readDatagram[3] = self.__calcCRC(readDatagram[:3])
        self.uart.write(bytearray(readDatagram))
        ech = self.uart.read(4)
        if bytearray(ech) != bytearray(readDatagram):
            print(ech)
        ret = self.uart.read(8)
        if ret[0] != 5:
            return -1
        else:
            return ret[3:7]

    def setPercent(self, motor, percent, direction ):
        bytespeed = self.__convert(percent, 0, 100, 0, 255 )
        if( motor == 'a' or motor == 'A' ):
            self.MOTOR_A_SPEED[1] = bytespeed
            if(direction == '+'):
                self.MOTOR_A_SPEED[0] = 0x00
            elif(direction == '-'):
                self.MOTOR_A_SPEED[0] = 0x01

        elif( motor == 'b' or motor == 'B' ):
            self.MOTOR_B_SPEED[1] = bytespeed
            if(direction == '+'):
                self.MOTOR_B_SPEED[0] = 0x00
            elif(direction == '-'):
                self.MOTOR_B_SPEED[0] = 0x01
        self.write_reg( PWM_AB, self.MOTOR_B_SPEED + self.MOTOR_A_SPEED )

