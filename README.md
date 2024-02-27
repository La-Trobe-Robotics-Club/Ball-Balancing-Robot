# Orientation-week-showcase-project

## OpenCV controls

You can control the maximum tilt of the platform. The default value is 5 degrees.

Hotkeys (Press in the actual openCV window)

### Q: Quits, also you can use the window close button in the window bar

### R: Reset disc position whilst in normal mode (not calibration or manual)

### P: toggle print output of motor forces and tilt multiplier to console

### S: toggle serial output of motor force to arduino

### C: Calibration mode, will only outline circles and do nothing else
When in calibration mode, press c again to start regular mode (start getting the disc and then lock it and get ball position etc)
Doesn't work in manual mode (nothing to calibrate as disc positon/radius set manually)

### M: Toggle manual mode, set the position and radius of the disc manually in the Trackbars Red window
There's a manual mode and automatic mode (via detecting circle of the disc color) to get the centerpoint and radius of the disk. Default is manual.

In manual mode, set the position and radius of the disc using the Trackbars.

## FOR DEVS
### Arduino
1. Open Arduino .ino file in the Arduino IDE.
2. Select Arduino UNO board.
### Serial
The arduino expects to recieve a byte for each motor in this order:
1. center motor
2. left motor
3. right motor

After recieving each byte it will send it back to the host computer (jetson nano), which indicates it is ready to recieve the next one.
Validate this byte before proceeding.

