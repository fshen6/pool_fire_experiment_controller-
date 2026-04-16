The goal of this system is to control the start and end procedures of my PhD experiment. This contains 2 individual controllers. 
Controller 1 controls the running of the peristaltic pump and the laser distance sensor.
The second controller is an off-the-shelf product, which controls the movement of the linear stages and ignition of my ignition coil.


Controller 1 is made from the following components:
esp32 wroom
3 stage toggle (D34 & D35)
10k potential meter (D32)
Voltage sensor (D33)
2 Relays (D19 & D25) Start/stop, Cw/CCW
3 buttons (D14 & D15 & D23) 
2x16 I2C LCD screen (D21 & D22)
E stop (D27)
PWM to voltage sensor, PWM out(D13)
RS485 to TTL (TX2->B & RX2->A & D4->REDE )

Relationship between the potentiometer, the voltage sensor and PWM to voltage converter.
A potentiometer is a medium for the user to interact with the machine and input the desired value from 0-100%. 
The PWM to voltage sensor is for converting the signal from the potentiometer and output PWM signal to be converted into voltage, which will control the rotation speed of the peristaltic pump.
The voltage sensor is connected parrallel in output of convertoer, to check the actual voltage output from the PWM to voltage converter, because this converter is not accurate.
So the potentiometer indirectly controls the voltage. The voltage sensor gets feedback to show the actual value.
potentiometer -> ESP32, ESP32->PWM to voltage convertor, Voltage convertor -> ESP32

Relay functions 
Relay1 Closed=start    Open=Stop
Relay2 Closed=CCW, Open=CW

The sensor is an RS-485 slave device that speaks Modbus RTU over a half-duplex differential bus. On the hardware side, the sensor’s RS-485 A and B lines are connected to a MAX485 transceiver, and the MAX485’s TTL side is connected to an Arduino Mega hardware UART, typically Serial1 (TX1 pin 18, RX1 pin 19). Because RS-485 is half-duplex, the MAX485 direction pins RE and DE are tied together and driven from one Arduino digital output: set that pin HIGH to enable transmit mode and LOW to enable receive mode. The sensor is powered separately, typically from 24 V, while the Arduino and MAX485 use 5 V, and all grounds must share a common reference if required by the transceiver/module arrangement. Communication is configured for 9600 baud, 8 data bits, no parity, 1 stop bit unless the sensor is reconfigured otherwise. To read the live measured distance, the master sends the Modbus RTU request frame 01 03 00 3B 00 02 B5 C6. Byte-by-byte, this means: slave address 0x01, function code 0x03 (Read Holding Registers), starting register address 0x003B, register count 0x0002, followed by the CRC16 checksum in little-endian order (0xB5 0xC6 as transmitted). After the frame is written to the UART, the master must wait for transmission to complete, switch the MAX485 back into receive mode, and then listen for the reply. A normal response has the form 01 03 04 D0 D1 D2 D3 CRC_L CRC_H, where 0x01 is the slave address, 0x03 is the echoed function code, 0x04 indicates four data bytes, D0..D3 are the payload bytes representing the measured distance, and the final two bytes are the Modbus CRC16. The payload is encoded as a 32-bit measurement value; according to the manual example for the 1 µm resolution model, a returned data field such as 00 00 B8 47 corresponds to hexadecimal 0x0000B847, decimal 47175, which represents 47175 µm, i.e. 47.175 mm. Therefore the software must reconstruct the 32-bit integer from the returned bytes using the correct byte order, then divide by 1000.0 to convert micrometres to millimetres. In code this is typically something like (uint32_t)D0 << 24 | (uint32_t)D1 << 16 | (uint32_t)D2 << 8 | D3, followed by distance_mm = raw_value / 1000.0. Robust decoding should also verify that the first byte matches the expected slave address, the second byte is the expected function code, the byte-count field is correct, the full expected frame length has been received before parsing, and ideally that the received CRC matches a freshly computed Modbus CRC16. Timing matters: the master must not remain in transmit mode after sending, or it will block the reply from the sensor; it must also allow enough response timeout for the slave to return the frame. If the application only wants to print data when the sensor actually replies, the program should suppress all serial output unless a complete, structurally valid response frame is received. In practical terms, the read cycle is: enable TX on MAX485, transmit the 8-byte Modbus query, flush UART, switch MAX485 to RX, buffer incoming bytes until the full response length is reached or a timeout occurs, validate the frame, parse the 32-bit measurement, convert from µm to mm, and only then print or act on the value.

Controller 1 is expected to do the following. 
Mode 1: manual prime mode
function: Allow the user to fill up or drain the container with precise readings from the distance sensor
Constantly read values from RS 485 and display it on LCD
prime: close relay 1 and generate maximum voltage (10v)
Close the circuit on Relay 1 and generate maximum voltage to start the pump with maximum speed.
if button 1 is pressed, close Relay 1 to Start; if the button is unpressed, open the circuit to stop
If Button 2 is pressed, close the Relay 2 and 1 to enter CCW mode and start, if button is unpressed, open both circuit to exit CCW mode and stop the pump
If E stop is pressed, close relay 1 and 2 and generate maximum voltage to strat the pump CCW until its unpressed.


Mode 2
Function: to manually micro-adjust the fuel injection and injection at a constant speed.
Keep reading data from RS485, and the potentiometer, convert the value from the potentiometer to PWM and show the voltage sensor data and display it on the LCD screen.
If button 1 is pressed, close the circuit on Relay 1, when it's unpressed, open Relay 1
If button 2 is pressed, close the circuit on Relay 2 and 1, and when it's unpressed, open Relay 1 and 2.
If Button 3 is pressed, close the Relay circuit 1 and keep it closed until pressed again.
If E stop is pressed, close relay 1 and 2 and generate maximum voltage to strat the pump CCW until its unpressed.


Mode 3
function: to automatically control the fuel injection according to the distance level sensor
constantly read from the distance sensor, the voltage sensor, the potentiometer and display them on LCD screen
When button3 is pressed. Save the voltage reading (VStart)and the distance reading(DStart), start the pump with VStart voltage output. Read distance sensor with sampling rate of 10HZ. 
Fuel level dropping = - disance 
Fuel level going up = +distance

if distance is = DStart, with +-0.05, keep the constant speed
Distance difference is < -0.1, increase the speed by 150% for 5 sampling time frame, reduce speed to VStart
Whenthe  difference is > 0.1, decrease the speed by 150% for 5 sampling time frame, reduce speed to VStart.

This cycle continues until button 3 is pressed again.



E stop 
ignore every previous instruction and
Close the Circuit 1, 2,3


