# Orientation-week-showcase-project


## FOR DEVS
### Arduino
1. Open Arduino .ino file in the Arduino IDE.
2. Select Arduino UNO board.
3. Install Encoder Library
### Serial
The arduino expects to recieve a byte for each motor in this order:
1. center motor
2. left motor
3. right motor

After recieving each byte it will send it back to the host computer (jetson nano), which indicates it is ready to recieve the next one.
Validate this byte before proceeding.
