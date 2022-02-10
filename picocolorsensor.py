# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

# Based on https://github.com/REVrobotics/Color-Sensor-v3/blob/main/src/main/java/com/revrobotics/ColorSensorV3.java
#
# Copyright (c) 2019 REV Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of REV Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import utime
import machine
import enum

def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """

    # Construct message
    msg = bytearray()
    msg.append(data)

    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)

class Color:

    def __init__(self, red: float, green: float, blue: float):
        self.red = red
        self.green = green
        self.blue = blue

class RawColor:

    def __init__(self, red: int, green: int, blue: int, ir: int):
        self.red = red
        self.green = green
        self.blue = blue
        self.ir = ir

class CIEColor:

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class ColorSensorV3:
    """REV Robotics Color Sensor V3"""

    kAddress = 0x52
    kPartID = 0xC2
    writeBuffer = bytearray([0])
    readBuffer = bytearray([0, 0, 0])

    def __init__(self):
        """
        Constructs a ColorSensor.

        port  The I2C port the color sensor is attached to
        """
        self.i2c = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16))

        if not self._checkDeviceID():
            return

        self._initializeDevice()

        # Clear the reset flag
        self.hasReset()

    class Register(enum.IntEnum):
        kMainCtrl = 0x00
        kProximitySensorLED = 0x01
        kProximitySensorPulses = 0x02
        kProximitySensorRate = 0x03
        kLightSensorMeasurementRate = 0x04
        kLightSensorGain = 0x05
        kPartID = 0x06
        kMainStatus = 0x07
        kProximityData = 0x08
        kDataInfrared = 0x0A
        kDataGreen = 0x0D
        kDataBlue = 0x10
        kDataRed = 0x13

    class MainControl(enum.IntFlag):
        kRGBMode = 0x04  # If bit is set to 1, color channels are activated
        kLightSensorEnable = 0x02  # Enable light sensor
        kProximitySensorEnable = 0x01  # Proximity sensor active
        OFF = 0x00  # Nothing on

    class GainFactor(enum.IntEnum):
        kGain1x = 0x00
        kGain3x = 0x01
        kGain6x = 0x02
        kGain9x = 0x03
        kGain18x = 0x04

    class LEDCurrent(enum.IntEnum):
        kPulse2mA = 0x00
        kPulse5mA = 0x01
        kPulse10mA = 0x02
        kPulse25mA = 0x03
        kPulse50mA = 0x04
        kPulse75mA = 0x05
        kPulse100mA = 0x06  # default value
        kPulse125mA = 0x07

    class LEDPulseFrequency(enum.IntEnum):
        kFreq60kHz = 0x18  # default value
        kFreq70kHz = 0x40
        kFreq80kHz = 0x28
        kFreq90kHz = 0x30
        kFreq100kHz = 0x38

    class ProximitySensorResolution(enum.IntEnum):
        kProxRes8bit = 0x00
        kProxRes9bit = 0x08
        kProxRes10bit = 0x10
        kProxRes11bit = 0x18

    class ProximitySensorMeasurementRate(enum.IntEnum):
        kProxRate6ms = 0x01
        kProxRate12ms = 0x02
        kProxRate25ms = 0x03
        kProxRate50ms = 0x04
        kProxRate100ms = 0x05  # default value
        kProxRate200ms = 0x06
        kProxRate400ms = 0x07

    class ColorSensorResolution(enum.IntEnum):
        kColorSensorRes20bit = 0x00
        kColorSensorRes19bit = 0x10
        kColorSensorRes18bit = 0x20
        kColorSensorRes17bit = 0x30
        kColorSensorRes16bit = 0x40
        kColorSensorRes13bit = 0x50

    class ColorSensorMeasurementRate(enum.IntEnum):
        kColorRate25ms = 0
        kColorRate50ms = 1
        kColorRate100ms = 2
        kColorRate200ms = 3
        kColorRate500ms = 4
        kColorRate1000ms = 5
        kColorRate2000ms = 7

    def configureProximitySensorLED(self, freq: LEDPulseFrequency,
                                    curr: LEDCurrent, pulses: int):
        """
        Configure the the IR LED used by the proximity sensor.

        These settings are only needed for advanced users, the defaults
        will work fine for most teams. Consult the APDS-9151 for more
        information on these configuration settings and how they will affect
        proximity sensor measurements.

        freq      The pulse modulation frequency for the proximity
                  sensor LED
        curr      The pulse current for the proximity sensor LED
        pulses    The number of pulses per measurement of the
                  proximity sensor LED (0-255)
        """
        self._write8(self.Register.kProximitySensorLED, freq | curr)
        self._write8(self.Register.kProximitySensorPulses, pulses)

    def configureProximitySensor(self, res: ProximitySensorResolution,
                                 rate: ProximitySensorMeasurementRate):
        """
        Configure the proximity sensor.

        These settings are only needed for advanced users, the defaults
        will work fine for most teams. Consult the APDS-9151 for more
        information on these configuration settings and how they will affect
        proximity sensor measurements.

        res   Bit resolution output by the proximity sensor ADC.
        rate  Measurement rate of the proximity sensor
        """
        self._write8(self.Register.kProximitySensorRate, res | rate)

    def configureColorSensor(self, res: ColorSensorResolution,
                             rate: ColorSensorMeasurementRate,
                             gain: GainFactor):
        """
        Configure the color sensor.

        These settings are only needed for advanced users, the defaults
        will work fine for most teams. Consult the APDS-9151 for more
        information on these configuration settings and how they will affect
        color sensor measurements.

        res   Bit resolution output by the respective light sensor ADCs
        rate  Measurement rate of the light sensor
        gain  Gain factor applied to light sensor (color) outputs
        """
        self._write8(self.Register.kLightSensorMeasurementRate, res | rate)
        self._write8(self.Register.kLightSensorGain, gain)

    def getColor(self) -> Color:
        """
        Get the most likely color. Works best when within 2 inches and
        perpendicular to surface of interest.

        Returns the most likely color, including unknown if
        the minimum threshold is not met
        """
        r = self.getRed()
        g = self.getGreen()
        b = self.getBlue()
        mag = r + g + b
        return Color(r / mag, g / mag, b / mag)

    def getProximity(self):
        """
        Get the raw proximity value from the sensor ADC (11 bit). This value
        is largest when an object is close to the sensor and smallest when
        far away.

        Returns proximity measurement value, ranging from 0 to 2047
        """
        return self._read11BitRegister(self.Register.kProximityData)

    def getRawColor(self) -> RawColor:
        """
        Get the raw color values from their respective ADCs (20-bit).

        Returns Color containing red, green, blue and IR values
        """
        return RawColor(self.getRed(), self.getGreen(), self.getBlue(),
                        self.getIR())

    def getRed(self) -> int:
        """
        Get the raw color value from the red ADC

        Returns Red ADC value
        """
        return self._read20BitRegister(self.Register.kDataRed)

    def getGreen(self) -> int:
        """
        Get the raw color value from the green ADC

        Returns Green ADC value
        """
        return self._read20BitRegister(self.Register.kDataGreen)

    def getBlue(self) -> int:
        """
        Get the raw color value from the blue ADC

        Returns Blue ADC value
        """
        return self._read20BitRegister(self.Register.kDataBlue)

    def getIR(self) -> int:
        """
        Get the raw color value from the IR ADC

        Returns IR ADC value
        """
        return self._read20BitRegister(self.Register.kDataInfrared)

    # This is a transformation matrix given by the chip
    # manufacturer to transform the raw RGB to CIE XYZ
    _Cmatrix = [
        0.048112847, 0.289453437, -0.084950826, -0.030754752, 0.339680186,
        -0.071569905, -0.093947499, 0.072838494, 0.34024948
    ]

    def getCIEColor(self) -> CIEColor:
        """
        Get the color converted to CIE XYZ color space using factory
        calibrated constants.

        https://en.wikipedia.org/wiki/CIE_1931_color_space

        Returns CIEColor value from sensor
        """
        raw = self.getRawColor()
        return CIEColor(
            self._Cmatrix[0] * raw.red + self._Cmatrix[1] * raw.green +
            self._Cmatrix[2] * raw.blue, self._Cmatrix[3] * raw.red +
            self._Cmatrix[4] * raw.green + self._Cmatrix[5] * raw.blue,
            self._Cmatrix[6] * raw.red + self._Cmatrix[7] * raw.green +
            self._Cmatrix[8] * raw.blue)

    def hasReset(self) -> bool:
        """
        Indicates if the device reset. Based on the power on status flag in the
        status register. Per the datasheet:

        Part went through a power-up event, either because the part was turned
        on or because there was power supply voltage disturbance (default at
        first register read).

        This flag is self clearing

        Returns bool indicating if the device was reset
        """
        self.i2c.readfrom_mem_into(self.kAddress, self.Register.kMainStatus, self.readBuffer)

        return (self.readBuffer[0] & 0x20) != 0

    def _checkDeviceID(self) -> bool:
        self.i2c.readfrom_mem_into(self.kAddress, self.Register.kPartID, self.readBuffer)

        if self.kPartID != self.readBuffer[0]:
            print(
                "Unknown device found with same I2C addres as REV color sensor")
            return False

        return True

    def _initializeDevice(self):
        self._write8(
            self.Register.kMainCtrl,
            self.MainControl.kRGBMode | self.MainControl.kLightSensorEnable |
            self.MainControl.kProximitySensorEnable)

        self._write8(
            self.Register.kProximitySensorRate,
            self.ProximitySensorResolution.kProxRes11bit |
            self.ProximitySensorMeasurementRate.kProxRate100ms)

        self._write8(self.Register.kProximitySensorPulses, 32)

    def _read11BitRegister(self, reg) -> int:
        self.i2c.readfrom_mem_into(self.kAddress, reg, self.readBuffer)

        return ((self.readBuffer[0] & 0xFF) | ((self.readBuffer[1] & 0xFF) << 8)) & 0x7FF

    def _read20BitRegister(self, reg) -> int:
        self.i2c.readfrom_mem_into(self.kAddress, reg, self.readBuffer)

        return ((self.readBuffer[0] & 0xFF) | ((self.readBuffer[1] & 0xFF) << 8) |
                ((self.readBuffer[2] & 0xFF) << 16)) & 0x03FFFF

    def _write8(self, reg, data: int):
        self.writeBuffer[0] = data
        self.i2c.writeto_mem(self.kAddress, reg, self.writeBuffer)


colorSensor = ColorSensorV3()
while True:
    rawColor = colorSensor.getRawColor()
    prox = colorSensor.getProximity()
    print("{", \
        rawColor.red, ",", \
        rawColor.green, ",", \
        rawColor.blue, ",", \
        rawColor.ir, ",", \
        prox, "}")

    utime.sleep_ms(100)
