The goal of this system is to control the start and end procedures of my PhD experiment. This contains 2 individual controllers. 
Controller 1 controls the running of the peristaltic pump, the laser distance sensor.
The second controller is an off the shelf product, it controls the movement of linear stages and ignition of my ignition coil.


Controller 1 is made from the following components:
esp32 wroom
3 stage toggle (D34 & D35)
10k potential meter (D32)
Voltage sensor (D33)
3 Mosfets (D19 & D25 & D26) Start/stop, Cw/CCW, Prime
3 buttons (D14 & D15 & D23) 
2x16 I2C LCD screen (D21 & D22)
E stop (D27)
PWM to voltage sensor, PWM out(D13)
RS485 to TTL (TX2->B & RX2->A & D4->REDE )

Controller 1 is expected to do the following. 
Mode 1: manual prime mode
function: Allow the user to fill up or drain the container with precise readings from distance sensor
Constantly read values from RS 485 and display it on LCD
Close the circuit on MOSFET 3 to enable prime mode
if button 1 is pressed, close MOSFET 1 to Start, if button is unpressed, oppen the circuit to stop
If Button 2 is pressed, Close the MOSFET 2 and 1 to enter CCW mode and start, if button is unpressed, open both circuit to exit CCW mode and stop the pump


Mode 2
Function: to manually micro-adjust the fuel injection and injection at a constant speed.
keep the mosfet 3 open, unlike mode 1
Keep reading data from RS485, and potential meter, convert value from Potentiometer to PWM and show voltage sensor data and display it on LCD screen.
If button 1 is pressed, close the circuit on mosfet 1, when its unpressed, open mosfet 1
If button 2 is pressed, close the circuit on Mosfet 2 and 1, and when its unpressed, open mosfet 1 and 2.
is Button 3 is pressed, close the mosfet circuit 1 and keep it closed until pressed again.


Mode 3


E stop 
ignore every previous instruction and
Close the Circuit 1, 2,3


