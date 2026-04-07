The goal of this system is to control the start and end procedures of my PhD experiment. This contains 2 individual controllers. 
Controller 1 controls the running of the peristaltic pump and the laser distance sensor.
The second controller is an off-the-shelf product, which controls the movement of the linear stages and ignition of my ignition coil.


Controller 1 is made from the following components:
esp32 wroom
3 stage toggle (D34 & D35)
10k potential meter (D32)
Voltage sensor (D33)
3 Relays (D19 & D25 & D26) Start/stop, Cw/CCW, Prime
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
Relay3 Closed=Prime, Open=no action


Controller 1 is expected to do the following. 
Mode 1: manual prime mode
function: Allow the user to fill up or drain the container with precise readings from the distance sensor
Constantly read values from RS 485 and display it on LCD
Close the circuit on Relay 3 to enable prime mode
if button 1 is pressed, close Relay 1 to Start, if the button is unpressed, open the circuit to stop
If Button 2 is pressed, close the Relay 2 and 1 to enter CCW mode and start, if button is unpressed, open both circuit to exit CCW mode and stop the pump


Mode 2
Function: to manually micro-adjust the fuel injection and injection at a constant speed.
keep the Relay 3 open, unlike mode 1
Keep reading data from RS485, and the potentiometer, convert the value from the potentiometer to PWM and show the voltage sensor data and display it on the LCD screen.
If button 1 is pressed, close the circuit on Relay 1, when it's unpressed, open Relay 1
If button 2 is pressed, close the circuit on Relay 2 and 1, and when it's unpressed, open Relay 1 and 2.
is Button 3 is pressed, close the Relay circuit 1 and keep it closed until pressed again.


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


