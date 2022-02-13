# REV Color Sensor Raspberry Pi Pico

This uses a Serial connection to transmit data for 2 Rev Color Sensors to a roboRIO using a Pi Pico.

## Pi Pico Setup

You can follow the raspberry pi instructions to build the executable in the pico folder. Their instructions are complicated for windows, so I have also provided a prebuilt uf2 file to directly upload to the device. To deploy this file this, hold down the button on your devide, and connect to your computer. This will open as a drive. Copy the picocolorsensor.uf2 file in the `pico` folder to the drive. It will automatically reboot and start running the code with no configuration required.

## Hardware connections

By default, the code will connect to 2 color sensors. For both sensors, Power is pin 36 and the easiest ground near that is pin 38.

Connect the first sensor's SDA to pin 6, and SCL to pin 7.

Connect the 2nd sensor's SDA to pin 9, and SCL to pin 10.

The roboRIO can provide Ground, 5v and the communication signal to the device.

* 5v on the roboRIO is MXP pin 1 (Top right pin), and connects to pin 39 of the Pi Pico.
* Gnd on the roboRIO is pin 8 (bottom row 4th in from the right) and connects to either Pin 3 or Pin 38 of the Pi Pico.
* UART Rx on the roboRIO is pin 10 (bottom row 5th in from the right) and connects to pin 1 of the Pi Pico.

## WPILib Setup

Copy the files corresponding to the language you use in the `wpilib` folder into your project. Instantiate a `PicoColorSensor` (java) or `pico::ColorSensor` (C++) object. You can only create 1 instance of this object. You can then use the functions to grab each color sensors value, and if its connected.
