# SPDX-FileCopyrightText: Copyright (c) 2024 Bradley Robert Carlile
#
# SPDX-License-Identifier: MIT
# MIT License
#
# Copyright (c) 2024 Bradley Robert Carlile
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE

"""
`bmp585`
================================================================================

MicroPython Driver for the Bosch BMP585 pressure sensors

* Author: Brad Carlile, Angelo Fatica

Based on

* micropython-bmp581/bmp581py. Author(s): Jose D. Montoya

"""
import time

from micropython import const
from i2c_helpers import CBits, RegisterStruct

try:
    import struct
except ImportError:
    import ustruct as struct

WORLD_AVERAGE_SEA_LEVEL_PRESSURE = 1013.25  # International average standard


class BMP:
    """Driver for the BMP585 Sensor connected over I2C.

    :param ~machine.I2C i2c: The I2C bus the BMP581 is connected to.
    :param int address: The I2C device address. Default :const:`0x47`, Secondary :const:`0x46`

    :raises RuntimeError: if the sensor is not found

    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`BMP581` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        from machine import Pin, I2C
        from micropython_bmpxxx import bmpxxx

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(1, sda=Pin(2), scl=Pin(3))
        bmp = bmpxxx.BMP581(i2c)

    Now you have access to the attributes

    .. code-block:: python

        press = bmp.pressure
        temp = bmp.temperature

        # altitude in meters based on sea level pressure of 1013.25 hPA
        meters = bmp.altitude
        print(f"alt = {meters:.2f} meters")

        # set sea level pressure to a known sea level pressure in hPa at nearest airport
        # https://www.weather.gov/wrh/timeseries?site=KPDX
        bmp.sea_level_pressure = 1010.80
        meters = bmp.altitude
        print(f"alt = {meters:.2f} meters")

        # Highest recommended resolution bmp581
        bmp.pressure_oversample_rate = bmp.OSR128
        bmp.temperature_oversample_rate = bmp.OSR8
        meters = bmp.altitude

    """

    # Power Modes for BMP581
    # in the BMP390 there is only SLEEP(aka STANDBY), FORCED, NORMAL
    STANDBY = const(0x00)
    NORMAL = const(0x01)
    FORCED = const(0x02)
    NON_STOP = const(0x03)
    power_mode_values = (STANDBY, NORMAL, FORCED, NON_STOP)

    # Oversample Rate
    OSR1 = const(0x00)
    OSR2 = const(0x01)
    OSR4 = const(0x02)
    OSR8 = const(0x03)
    OSR16 = const(0x04)
    OSR32 = const(0x05)
    OSR64 = const(0x06)
    OSR128 = const(0x07)

    # oversampling rates
    pressure_oversample_rate_values = (OSR1, OSR2, OSR4, OSR8, OSR16, OSR32, OSR64, OSR128)
    temperature_oversample_rate_values = (OSR1, OSR2, OSR4, OSR8, OSR16, OSR32, OSR64, OSR128)

    # IIR Filters Coefficients
    COEF_0 = const(0x00)
    COEF_1 = const(0x01)
    COEF_3 = const(0x02)
    COEF_7 = const(0x03)
    COEF_15 = const(0x04)
    COEF_31 = const(0x05)
    COEF_63 = const(0x06)
    COEF_127 = const(0x07)
    iir_coefficient_values = (COEF_0, COEF_1, COEF_3, COEF_7, COEF_15, COEF_31, COEF_63, COEF_127)

    BMP581_I2C_ADDRESS_DEFAULT = 0x47
    BMP581_I2C_ADDRESS_SECONDARY = 0x46

    # bmp581 Address & Settings
    _REG_WHOAMI = const(0x01)
    _INT_STATUS = const(0x27)
    _INT_SOURCE = const(0x15)
    _DSP_CONFIG = const(0x30)
    _DSP_IIR = const(0x31)
    _OSR_CONF = const(0x36)
    _ODR_CONFIG = const(0x37)
    _CMD_BMP581 = const(0x7e)

    _device_id = RegisterStruct(_REG_WHOAMI, "B")
    _SOFTRESET = const(0xB6)  # same value for 585,581,390,280

    _cmd_register_BMP581 = CBits(8, _CMD_BMP581, 0)
    _enable_drdy = CBits(1, _INT_SOURCE, 0)
    _drdy_status = CBits(1, _INT_STATUS, 0)
    _power_mode = CBits(2, _ODR_CONFIG, 0)
    _temperature_oversample_rate = CBits(3, _OSR_CONF, 0)
    _pressure_oversample_rate = CBits(3, _OSR_CONF, 3)
    _output_data_rate = CBits(5, _ODR_CONFIG, 2)
    _pressure_enabled = CBits(1, _OSR_CONF, 6)
    _iir_coefficient = CBits(3, _DSP_IIR, 3)  # Pressure IIR coefficient
    _iir_temp_coefficient = CBits(3, _DSP_IIR, 0)  # Temp IIR coefficient
    _iir_control = CBits(8, _DSP_CONFIG, 0)
    _temperature = CBits(24, 0x1D, 0, 3)
    _pressure = CBits(24, 0x20, 0, 3)

    def __init__(self, i2c, address: int = None) -> None:
        time.sleep_ms(3)  # t_powup done in 2ms

        # If no address is provided, try the default, then secondary
        if address is None:
            if self._check_address(i2c, self.BMP581_I2C_ADDRESS_DEFAULT):
                address = self.BMP581_I2C_ADDRESS_DEFAULT
            elif self._check_address(i2c, self.BMP581_I2C_ADDRESS_SECONDARY):
                address = self.BMP581_I2C_ADDRESS_SECONDARY
            else:
                raise RuntimeError("BMP581 sensor not found at I2C expected address (0x47,0x46).")
        else:
            # Check if the specified address is valid
            if not self._check_address(i2c, address):
                raise RuntimeError(f"BMP581 sensor not found at specified I2C address ({hex(address)}).")

        self._i2c = i2c
        self._address = address
        if self._read_device_id() != 0x50:  # check _device_id after i2c established
            raise RuntimeError("Failed to find the BMP581 sensor")

        self._cmd_register_BMP581 = _SOFTRESET
        time.sleep_ms(5)  # soft reset finishes in 2ms

        # Must be in STANDBY to initialize _iir_coefficient    
        self._power_mode = STANDBY
        time.sleep_ms(5)  # mode change takes 4ms
        self._pressure_enabled = True
        self._output_data_rate = 0  # Default rate
        self._temperature_oversample_rate = self.OSR1  # Default oversampling
        self._pressure_oversample_rate = self.OSR1  # Default oversampling
        self._iir_coefficient = COEF_0
        self._iir_temp_coefficient = COEF_0
        self._enable_drdy = True
        time.sleep_ms(5)  # mode change takes 4ms

#         self._drdy_status = 0  # Default data-ready status
        self.sea_level_pressure = WORLD_AVERAGE_SEA_LEVEL_PRESSURE

    def _check_address(self, i2c, address: int) -> bool:
        """Helper function to check if a device responds at the given I2C address."""
        try:
            i2c.writeto(address, b"")  # Attempt a write operation
            return True
        except OSError:
            return False

    def _read_device_id(self) -> int:
        return self._device_id

    @property
    def config(self):
        print(f"{hex(self._address)=}")
        print(f"{hex(self._device_id)=}")
        print(f"{self.power_mode=}")
        print(f"{self.pressure_oversample_rate=}")
        print(f"{self.temperature_oversample_rate=}")
        print(f"{self.iir_coefficient=}")
        print(f"{self.sea_level_pressure=}")
        print(f"{self.pressure=} hPa")
        print(f"{self.temperature=} C")
        print(f"{self.altitude=} m\n")

    @property
    def power_mode(self) -> str:
        """
        Sensor power_mode
        +-----------------------------+------------------+
        | Mode                        | Value            |
        +=============================+==================+
        | :py:const:`bmp58x.STANDBY`  | :py:const:`0x00` |
        | :py:const:`bmp58x.NORMAL`   | :py:const:`0x01` |
        | :py:const:`bmp58x.FORCED`   | :py:const:`0x02` |
        | :py:const:`bmp58x.NON_STOP` | :py:const:`0X03` |
        +-----------------------------+------------------+
        """
        values = ("STANDBY", "NORMAL", "FORCED", "NON_STOP",)
        return values[self._power_mode]

    @power_mode.setter
    def power_mode(self, value: int) -> None:
        if value not in self.power_mode_values:
            raise ValueError("Value must be a valid power_mode setting: STANDBY,NORMAL,FORCED,NON_STOP")
        self._power_mode = value

    @property
    def pressure_oversample_rate(self) -> str:
        """
        Sensor pressure_oversample_rate
        Oversampling extends the measurement time per measurement by the oversampling
        factor. Higher oversampling factors offer decreased noise at the cost of
        higher power consumption.
        +---------------------------+------------------+
        | Mode                      | Value            |
        +===========================+==================+
        | :py:const:`bmpxxx.OSR1`   | :py:const:`0x00` |
        | :py:const:`bmpxxx.OSR2`   | :py:const:`0x01` |
        | :py:const:`bmpxxx.OSR4`   | :py:const:`0x02` |
        | :py:const:`bmpxxx.OSR8`   | :py:const:`0x03` |
        | :py:const:`bmpxxx.OSR16`  | :py:const:`0x04` |
        | :py:const:`bmpxxx.OSR32`  | :py:const:`0x05` |
        | :py:const:`bmp58x.OSR64`  | :py:const:`0x06` |
        | :py:const:`bmp58x.OSR128` | :py:const:`0x07` |
        +---------------------------+------------------+
        :return: sampling rate as string
        """
        string_name = ("OSR1", "OSR2", "OSR4", "OSR8", "OSR16", "OSR32", "OSR64", "OSR128",)
        return string_name[self._pressure_oversample_rate]

    @pressure_oversample_rate.setter
    def pressure_oversample_rate(self, value: int) -> None:
        if value not in self.pressure_oversample_rate_values:
            raise ValueError(
                "Value must be a valid pressure_oversample_rate: OSR1,OSR2,OSR4,OSR8,OSR16,OSR32,OSR64,OSR128")
        self._pressure_oversample_rate = value

    @property
    def temperature_oversample_rate(self) -> str:
        """
        Sensor temperature_oversample_rate
        +---------------------------+------------------+
        | Mode                      | Value            |
        +===========================+==================+
        | :py:const:`bmpxxx.OSR1`   | :py:const:`0x00` |
        | :py:const:`bmpxxx.OSR2`   | :py:const:`0x01` |
        | :py:const:`bmpxxx.OSR4`   | :py:const:`0x02` |
        | :py:const:`bmpxxx.OSR8`   | :py:const:`0x03` |
        | :py:const:`bmpxxx.OSR16`  | :py:const:`0x04` |
        | :py:const:`bmpxxx.OSR32`  | :py:const:`0x05` |
        | :py:const:`bmp58x.OSR64`  | :py:const:`0x06` |
        | :py:const:`bmp58x.OSR128` | :py:const:`0x07` |
        +---------------------------+------------------+
        :return: sampling rate as string
        """
        string_name = ("OSR1", "OSR2", "OSR4", "OSR8", "OSR16", "OSR32", "OSR64", "OSR128",)
        return string_name[self._temperature_oversample_rate]

    @temperature_oversample_rate.setter
    def temperature_oversample_rate(self, value: int) -> None:
        if value not in self.temperature_oversample_rate_values:
            raise ValueError(
                "Value must be a valid temperature_oversample_rate: OSR1,OSR2,OSR4,OSR8,OSR16,OSR32,OSR64,OSR128")
        self._temperature_oversample_rate = value

    @property
    def temperature(self) -> float:
        """
        :return: Temperature in Celsius
        """
        self._power_mode = FORCED
        time.sleep_ms(5)
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 50:
            st = self._drdy_status
            if st:
                raw_temp = self._temperature
                return self._twos_comp(raw_temp, 24) / 65536.0
        raise RuntimeError("BMP585: data_ready timeout")

    @property
    def pressure(self) -> float:
        """
        :return: Pressure
        """
        self._power_mode = FORCED
        time.sleep_ms(5)
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 50:
            st = self._drdy_status
            if st:
                pressure = self._twos_comp(self._pressure, 24) / 64.0 / 100.0
                inh2o = pressure*0.40146307866177 - self.offset
                self.raw_prs = pressure*0.40146307866177
                self.inh2o = inh2o

                if self.unit == "inh2o":
                    return inh2o
                elif self.unit == "kpa":
                    return inh2o*0.24884
                elif self.unit == "mbar":
                    return inh2o*2.4884
                elif self.unit == "psi":
                    return inh2o*0.0360912
        raise RuntimeError("BMP585: data_ready timeout")

    @property
    def altitude(self) -> float:
        """
        Using the sensor's measured pressure and the pressure at sea level (e.g., 1013.25 hPa),
        the altitude in meters is calculated with the international barometric formula
        https://ncar.github.io/aircraft_ProcessingAlgorithms/www/PressureAltitude.pdf
        """
        altitude = 44330.77 * (
                1.0 - ((self.pressure / self.sea_level_pressure) ** 0.1902632)
        )
        return altitude

    @altitude.setter
    def altitude(self, value: float) -> None:
        self.sea_level_pressure = self.pressure / (1.0 - value / 44330.77) ** (1 / 0.1902632)

    @property
    def sea_level_pressure(self) -> float:
        """
        :return: Sea-level pressure in hPa
        """
        return self._sea_level_pressure

    @sea_level_pressure.setter
    def sea_level_pressure(self, value: float) -> None:
        self._sea_level_pressure = value

    @staticmethod
    def _twos_comp(val: int, bits: int) -> int:
        if val & (1 << (bits - 1)) != 0:
            return val - (1 << bits)
        return val

    @property
    def iir_coefficient(self) -> str:
        """
        Sensor iir_coefficient
        +----------------------------+------------------+
        | Mode                       | Value            |
        +============================+==================+
        | :py:const:`bmpxxx.COEF_0`  | :py:const:`0x00` |
        | :py:const:`bmpxxx.COEF_1`  | :py:const:`0x01` |
        | :py:const:`bmpxxx.COEF_3`  | :py:const:`0x02` |
        | :py:const:`bmpxxx.COEF_7`  | :py:const:`0x03` |
        | :py:const:`bmpxxx.COEF_15` | :py:const:`0x04` |
        | :py:const:`bmpxxx.COEF_31` | :py:const:`0x05` |
        | :py:const:`bmpxxx.COEF_63` | :py:const:`0x06` |
        | :py:const:`bmpxxx.COEF_127`| :py:const:`0x07` |
        +----------------------------+------------------+
        :return: coefficients as string
        """
        values = ("COEF_0", "COEF_1", "COEF_3", "COEF_7", "COEF_15", "COEF_31", "COEF_63", "COEF_127",)
        return values[self._iir_coefficient]

    @iir_coefficient.setter
    def iir_coefficient(self, value: int) -> None:
        if value not in self.iir_coefficient_values:
            raise ValueError(
                "Value must be a valid iir_coefficients: COEF_0,COEF_1,COEF_3,COEF_7,COEF_15,COEF_31,COEF_63,COEF_127")

        # Ensure the sensor is in STANDBY mode before updating
        original_mode = self._power_mode  # Save the current mode
        if original_mode != STANDBY:
            self.power_mode = STANDBY  # Set to STANDBY if not already
        self._iir_coefficient = value
        self._iir_temp_coefficient = value

        # Restore the original power mode
        self.power_mode = original_mode

    @property
    def output_data_rate(self) -> int:
        """
        Sensor output_data_rate. for a complete list of values please see the datasheet
        """
        return self._output_data_rate

    @output_data_rate.setter
    def output_data_rate(self, value: int) -> None:
        if value not in range(0, 32, 1):
            raise ValueError("Value must be a valid output_data_rate setting: 0 to 32")
        self._output_data_rate = value


class BMP585(BMP):
    """Driver for the BMP585 Sensor connected over I2C.

    :param ~machine.I2C i2c: The I2C bus the BMP585 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x47`

    :raises RuntimeError: if the sensor is not found

    **Quickstart: Importing and using the device**

    .. code-block:: python

        from machine import Pin, I2C
        import bmp585

    Once this is done you can define your `machine.I2C` object and define your sensor object

    .. code-block:: python

        i2c = I2C(1, sda=Pin(2), scl=Pin(3))
        bmp = bmpxxx.BMP585(i2c)

    Now you have access to the attributes

    .. code-block:: python

        press = bmp.pressure
        temp = bmp.temperature

        # Highest recommended resolution for bmp585
        bmp.pressure_oversample_rate = bmp.OSR128
        bmp.temperature_oversample_rate = bmp.OSR8
        meters = bmp.altitude

    """
    BMP585_I2C_ADDRESS_DEFAULT = 0x47
    BMP585_I2C_ADDRESS_SECONDARY = 0x46

    _CMD_BMP585 = const(0x7e)
    _cmd_register_BMP585 = CBits(8, _CMD_BMP585, 0)

    def __init__(self, i2c, channel, address, mux_address, offset):
        time.sleep_ms(3)  # t_powup done in 2ms
        # Check if the specified address is valid
        if not self._check_address(i2c, address):
            raise RuntimeError(f"BMP585 sensor not found at specified I2C address ({hex(address)}).")
        self._i2c = i2c
        self._address = address
        if self._read_device_id() != 0x51:  # check _device_id after i2c established
            raise RuntimeError("Failed to find the BMP585 sensor")
        self._cmd_register_BMP585 = _SOFTRESET
        time.sleep_ms(5)  # soft reset finishes in 2ms
        # Must be in STANDBY to initialize _iir_coefficient
        self._power_mode = STANDBY
        time.sleep_ms(5)  # mode change takes 4ms
        self._pressure_enabled = True
        self._temperature_oversample_rate = self.OSR1  # Default oversampling
        self._pressure_oversample_rate = self.OSR1  # Default oversampling
        self._iir_coefficient = COEF_0
        self._iir_temp_coefficient = COEF_0
        self._enable_drdy = True
        self.sea_level_pressure = WORLD_AVERAGE_SEA_LEVEL_PRESSURE
        self.mux_address = mux_address
        self.offset = offset
        self.unit = "inh2o"
        self.inh2o = 0
        self.raw_prs = 0
        self.channel = channel
        self.output_data_rate = 0

    def change_unit(self, unit):
        self.unit = unit
