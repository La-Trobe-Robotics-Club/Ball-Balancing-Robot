import serial
import cv2
import numpy as np
import platform
import time
import math

#ADD SECOND PARAMETER
#def activate_motors(ball_direction_degrees, power):
#    angle = ball_direction_degrees % 360
#    section = int(angle // 60) + 1
#    
#    #REPLACE EACH print() with a call to each motor, parsing power
#    if section == 1:
#        print(f"MOTOR1 {power}")
#        center_motor = int(power) #1
#    elif section == 2:
#        print(f"MOTOR1 {power}", end="")
#        center_motor = int(power) #1
#        print(f" MOTOR3 {power}")
#        right_motor = int(power) #3
#    elif section == 3:
#        print(f"MOTOR3 {power}")
#        right_motor = int(power) #3
#    elif section == 4:
#        print(f"MOTOR3 {power}", end="")
#        right_motor = int(power) #3
#        print(f" MOTOR5 {power}")
#        left_motor = int(power) #5
#    elif section == 5:
#        print(f"MOTOR5 {power}", end="")
#        left_motor = int(power) #5
#    elif section == 6:
#        print(f"MOTOR5 {power}", end="")
#        left_motor = int(power) #5
#        print(f" MOTOR1 {power}")
#        center_motor = int(power) #1

#exponentially returns a % power for the motor to activate at the less time the ball has from the edge
def get_edge_elevation(time_to_edge):
    if time_to_edge <= 1:
        return 100
    if time_to_edge >= 8:
        return 0
    decay_base = 0.5
    elapsed_steps = time_to_edge - 1
    return 100 * (decay_base ** elapsed_steps)


def nothing(x):
    pass

# Function to detect circles based on color
def detect_circles(frame, lower_color, upper_color, min_radius, max_radius):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
    cv2.imshow('mask', mask) # show what the function sees to assist calibration
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=min_radius, maxRadius=max_radius)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        largest_circle = sorted(circles[0, :], key=lambda x: x[2], reverse=True)[0]
        return (largest_circle[0], largest_circle[1], largest_circle[2])  # Return the largest circle's center and radius
    return None

# Calculate the distance between two points
def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def angle3pt(right_line, ball_point):
    right_point = right_line[1]
    center = right_line[0]
    """Counterclockwise angle in degrees by turning from a to c around b
        Returns a float between 0.0 and 360.0"""
    ang = np.degrees(
        np.arctan2(ball_point[1] - center[1], ball_point[0] - center[0]) - np.arctan2(right_point[1] - center[1], right_point[0] - center[0]))
    return ang + 360 if ang < 0 else ang

# Calculate an endpoint of a line given origin, degrees, and length)
def calculate_line_endpoint(origin, degrees, length):
    angle_radians = np.radians(degrees)
    x2 = int(origin[0] + length * np.cos(angle_radians))
    y2 = int(origin[1] + length * np.sin(angle_radians))
    return (x2, y2)

def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

# Code starts here
# Serial config
serial_output_string = input("Use serial and initialize arduino connection? (y/n) ")
serial_output = False
serial_initialised = False
center_motor = 0
left_motor = 0
right_motor = 0
serial_count = 0
serial_sent = None
serial_recieved = None
calibrated = False
calibration_input = None
arduino_calibration_output = None
if serial_output_string == "y":
    #ser = serial.Serial('/dev/ttyUSB0')
    ser = serial.Serial('COM8')
    # Wait for arduino to be ready
    print("Waiting for Arduino")
    ser.read_until("STARTED\r\n", 11)
    print("Arduino ready to calibrate")
    while not calibrated:
        arduino_calibration_output = ser.read_until().decode()
        print(f"{arduino_calibration_output}")
        calibration_input = input("Full rotation:23945.84|")
        ser.write((calibration_input + "|").encode())
        if calibration_input == "y":
            calibrated = True
        arduino_calibration_output = ser.read_until().decode()
        print(f"{arduino_calibration_output}")

    ser.write(int.to_bytes(center_motor))
    serial_sent = center_motor
    serial_count = 1
    print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")
    serial_initialised = True
    serial_output = True

#OS info
platform_os = platform.system()

# Start capturing video
if platform_os == "Windows":
    cap = cv2.VideoCapture(3, cv2.CAP_DSHOW)
else:
    cap = cv2.VideoCapture(0)

ret, frame = cap.read()
cv2.imshow('Frame', frame)
cv2.namedWindow('trackbar')
# Create a trackbar to set the maximum angle of tilt and rotation of disc to align motors
cv2.createTrackbar('MaxTilt', 'trackbar', 0, 255, nothing) # Don't move until this slider is moved
cv2.createTrackbar('Rotation', 'trackbar', 149, 360, nothing)
# Create a set of trackbars for manual adjustment of center
cv2.createTrackbar('ManPosX', 'trackbar', 278, 800, nothing)
cv2.createTrackbar('ManPosY', 'trackbar', 169, 450, nothing)
cv2.createTrackbar('ManRadius', 'trackbar', 292, 500, nothing)
# Create a set of trackbars for PID
cv2.createTrackbar('kP', 'trackbar', 255, 255, nothing)
cv2.createTrackbar('kI', 'trackbar', 0, 1000, nothing)
cv2.createTrackbar('kD', 'trackbar', 0, 1000, nothing)
# Create a set of trackbars for  HSV adjustment of center red disk
# cv2.createTrackbar('Lower Hue Disk', 'Frame', 0, 179, nothing)
# cv2.createTrackbar('Upper Hue Disk', 'Frame', 179, 179, nothing)
# cv2.createTrackbar('Lower Saturation Disk', 'Frame', 100, 255, nothing)
# cv2.createTrackbar('Upper Saturation Disk', 'Frame', 255, 255, nothing)
# cv2.createTrackbar('Lower Value Disk', 'Frame', 100, 255, nothing)
# cv2.createTrackbar('Upper Value Disk', 'Frame', 255, 255, nothing)
# cv2.createTrackbar('Min Radius Disk', 'Frame', 0, 150, nothing)
# cv2.createTrackbar('Max Radius Disk', 'Frame', 0, 150, nothing)

# Create another set of trackbars for HSV adjustment for the ball
# Calibrated for yellow ball in Les's bedroom
cv2.createTrackbar('Lower Hue Ball', 'trackbar', 5, 179, nothing) 
cv2.createTrackbar('Upper Hue Ball', 'trackbar', 90, 179, nothing)
cv2.createTrackbar('Lower Saturation Ball', 'trackbar', 80, 255, nothing)
cv2.createTrackbar('Upper Saturation Ball', 'trackbar', 255, 255, nothing)
cv2.createTrackbar('Lower Value Ball', 'trackbar', 59, 255, nothing)
cv2.createTrackbar('Upper Value Ball', 'trackbar', 255, 255, nothing)
cv2.createTrackbar('Min Radius Ball', 'trackbar', 20, 150, nothing)
cv2.createTrackbar('Max Radius Ball', 'trackbar', 60, 150, nothing)


# Mark's colors, red and black, for testing purposes only, red and yellow wasn't working
# # # Create windows with trackbars for HSV adjustment for the red circle
# cv2.namedWindow('Frame')
# cv2.createTrackbar('Lower Hue', 'Frame', 0, 179, nothing)
# cv2.createTrackbar('Upper Hue', 'Frame', 179, 179, nothing)
# cv2.createTrackbar('Lower Saturation', 'Frame', 100, 255, nothing)
# cv2.createTrackbar('Upper Saturation', 'Frame', 255, 255, nothing)
# cv2.createTrackbar('Lower Value', 'Frame', 100, 255, nothing)
# cv2.createTrackbar('Upper Value', 'Frame', 255, 255, nothing)
# cv2.createTrackbar('Min Radius', 'Frame', 188, 300, nothing)
# cv2.createTrackbar('Max Radius', 'Frame', 204, 300, nothing)

# # Create another set of trackbars for HSV adjustment for the black circle
# cv2.namedWindow('Frame')
# cv2.createTrackbar('Lower Hue', 'Frame', 0, 179, nothing)  # Adjusted for fluorescent yellow
# cv2.createTrackbar('Upper Hue', 'Frame', 179, 179, nothing)
# cv2.createTrackbar('Lower Saturation', 'Frame', 0, 255, nothing)
# cv2.createTrackbar('Upper Saturation', 'Frame', 255, 255, nothing)
# cv2.createTrackbar('Lower Value', 'Frame', 0, 255, nothing)
# cv2.createTrackbar('Upper Value', 'Frame', 109, 255, nothing)
# cv2.createTrackbar('Min Radius', 'Frame', 47, 150, nothing)
# cv2.createTrackbar('Max Radius', 'Frame', 72, 150, nothing)

cv2.createTrackbar("CAP_PROP_AUTOFOCUS", 'trackbar', 0, 1, nothing)
cv2.createTrackbar("CAP_PROP_FOCUS", 'trackbar', 5, 255, nothing)
cv2.createTrackbar("CAP_PROP_CONTRAST", 'trackbar', 128, 255, nothing)
cv2.createTrackbar("CAP_PROP_BRIGHTNESS", 'trackbar', 128, 255, nothing)
cv2.createTrackbar("CAP_PROP_SATURATION", 'trackbar', 128, 255, nothing)
cv2.createTrackbar("CAP_PROP_EXPOSURE", 'trackbar', 511, 1023, nothing)
'''
cap.set(cv2.cv.CAP_PROP_AUTOFOCUS, 0.) 
cap.set(cv2.cv.CAP_PROP_FOCUS, 5.)
cap.set(cv2.cv.CAP_PROP_CONTRAST, 128.)
cap.set(cv2.cv.CAP_PROP_BRIGHTNESS, 128.)
cap.set(cv2.cv.CAP_PROP_SATURATION, 128.)
cap.set(cv2.cv.CAP_PROP_CONVERT_RGB, 1.) 
cap.set(cv2.cv.CAP_PROP_EXPOSURE, 511.) 
'''

get_disc = True
calibrate_mode = False
manual_mode = True
disc_centers = []
disc_center = (-1, -1)
right_line = ((-1, -1), (-1, -1))
radii = []
radius = -1
rotation = 0
DISC_AVG_SAMPLE_SIZE = 40
NUM_MOTORS = 3
ball_mom =[]
mom_amount = 0

force_multiplier = 0
angle = 0
dist_center = 0

print_output = True

motor_outputs = [0] * NUM_MOTORS

center_motor = 0
left_motor = 0
right_motor = 0

seg_size = 360 / NUM_MOTORS
segment_angles = np.linspace(0,-360,NUM_MOTORS,endpoint=False)
indexes_and_segment_angles = [(i, int(a)) for i, a in enumerate(segment_angles)]
indexes_and_segment_angles.append((0, 360))

last_dist = [None, None, None]
derivative_update_time = [None, None, None]
while True:
    # set camera settings
    cap.set(cv2.CAP_PROP_FOCUS, cv2.getTrackbarPos('CAP_PROP_FOCUS', 'trackbar'))
    cap.set(cv2.CAP_PROP_CONTRAST, cv2.getTrackbarPos('CAP_PROP_CONTRAST', 'trackbar'))
    cap.set(cv2.CAP_PROP_BRIGHTNESS, cv2.getTrackbarPos('CAP_PROP_BRIGHTNESS', 'trackbar'))
    cap.set(cv2.CAP_PROP_SATURATION, cv2.getTrackbarPos('CAP_PROP_SATURATION', 'trackbar'))
    cap.set(cv2.CAP_PROP_EXPOSURE, cv2.getTrackbarPos('CAP_PROP_EXPOSURE', 'trackbar'))

    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera. Check camera index and connection.")
        break
    if manual_mode:
        trackbar_radius = cv2.getTrackbarPos('ManRadius', 'trackbar')
        trackbar_center = (cv2.getTrackbarPos('ManPosX', 'trackbar'), cv2.getTrackbarPos('ManPosY', 'trackbar'))
        trackbar_rotation = cv2.getTrackbarPos('Rotation', 'trackbar')
        if trackbar_radius != radius or trackbar_center != disc_center or trackbar_rotation != rotation:
            radius = trackbar_radius
            disc_center = trackbar_center
            rotation = trackbar_rotation
            segment_endpoints = []
            endpoint_up = calculate_line_endpoint(disc_center, segment_angles[0]-90, radius)
            
            endpoint_right = calculate_line_endpoint(disc_center, segment_angles[0], radius)
            right_line = (disc_center, endpoint_right)
            
            for a in segment_angles:
                segment_endpoints.append(calculate_line_endpoint(disc_center, a+rotation, radius))
        get_disc = False
    elif get_disc or calibrate_mode:
        # Get trackbar positions for red detections
        lower_red = np.array([cv2.getTrackbarPos('Lower Hue Disk', 'trackbar'), cv2.getTrackbarPos('Lower Saturation Disk', 'trackbar'), cv2.getTrackbarPos('Lower Value Disk', 'trackbar')])
        upper_red = np.array([cv2.getTrackbarPos('Upper Hue Disk', 'trackbar'), cv2.getTrackbarPos('Upper Saturation Disk', 'trackbar'), cv2.getTrackbarPos('Upper Value Disk', 'trackbar')])
        min_radius_red = cv2.getTrackbarPos('Min Radius Disk', 'trackbar')
        max_radius_red = cv2.getTrackbarPos('Max Radius Disk', 'trackbar')
        # Detect red circles
        red_circle = detect_circles(frame, lower_red, upper_red, min_radius_red, max_radius_red)
        if red_circle is not None:
            cv2.circle(frame, (red_circle[0], red_circle[1]), red_circle[2], (0, 255, 0), 2)
            cv2.circle(frame, (red_circle[0], red_circle[1]), 2, (0, 0, 255), 3)  # Center point
            if not calibrate_mode:
                disc_centers.append((red_circle[0], red_circle[1]))
                radii.append(red_circle[2])
        if len(disc_centers) == DISC_AVG_SAMPLE_SIZE:
            mean = np.mean(disc_centers,axis=0)
            disc_center = (int(mean[0]), int(mean[1]))
            disc_centers = []
            radius = int(sum(radii) / len(radii))
            radii = []
            segment_endpoints = []
            endpoint_up = calculate_line_endpoint(disc_center, segment_angles[0]-90, radius)
            
            endpoint_right = calculate_line_endpoint(disc_center, segment_angles[0], radius)
            right_line = (disc_center, endpoint_right)
            trackbar_rotation = cv2.getTrackbarPos('Rotation', 'trackbar')
            rotation = trackbar_rotation
            for a in segment_angles:
                segment_endpoints.append(calculate_line_endpoint(disc_center, a+rotation, radius))
            
            get_disc = False
    if not get_disc or calibrate_mode:
        # Draw averaged circle and center dot
        # Detect yellow circles
        lower_yellow = np.array([cv2.getTrackbarPos('Lower Hue Ball', 'trackbar'), cv2.getTrackbarPos('Lower Saturation Ball', 'trackbar'), cv2.getTrackbarPos('Lower Value Ball', 'trackbar')])
        upper_yellow = np.array([cv2.getTrackbarPos('Upper Hue Ball', 'trackbar'), cv2.getTrackbarPos('Upper Saturation Ball', 'trackbar'), cv2.getTrackbarPos('Upper Value Ball', 'trackbar')])
        min_radius_yellow = cv2.getTrackbarPos('Min Radius Ball', 'trackbar')
        max_radius_yellow = cv2.getTrackbarPos('Max Radius Ball', 'trackbar')
        ball_circle = detect_circles(frame, lower_yellow, upper_yellow, min_radius_yellow, max_radius_yellow)

        # Draw lines representing the segments
        if not calibrate_mode:
            cv2.line(frame, disc_center, segment_endpoints[0], (0, 255, 0), 3) # Green
            cv2.line(frame, disc_center, segment_endpoints[1], (0, 0, 255), 3) # Red
            cv2.line(frame, disc_center, segment_endpoints[2], (255, 0, 0), 3) # Blue
            for endpoint in segment_endpoints[3:]:
                cv2.line(frame, disc_center, endpoint, (0, 0, 0), 3)
            cv2.circle(frame, disc_center, radius, (0, 255, 0), 2)
            cv2.circle(frame, disc_center, 2, (0, 0, 255), 3)

        if ball_circle is not None:
            ball_center = (int(ball_circle[0]), int(ball_circle[1]))
            ball_radius = ball_circle[2]
            cv2.circle(frame, ball_center, ball_radius, (0, 255, 255), 2)
            cv2.circle(frame, ball_center, 2, (255, 255, 255), 3)  # Center point
            ball_mom.append(ball_center)
            for cord_count in ball_mom :
                mom_amount += 1
            if mom_amount == 5:
                ball_will = (ball_mom[0] + (ball_mom[5] - ball_mom [0]))
                ball_mom.pop(0)
                print(ball_will)
                cv2.line(frame, ball_center, ball_will, (0, 0, 255), 3)
                mom_amount -= 1

            if print_output or serial_output:
                dist_to_motors = []
                for e in segment_endpoints:
                    dist_to_motors.append(calculate_distance(e, ball_center))
                motor_outputs = []
                tilt_multiplier = cv2.getTrackbarPos('MaxTilt', 'trackbar') / 255
                kP = cv2.getTrackbarPos('kP', 'trackbar') / 255
                kI = cv2.getTrackbarPos('kI', 'trackbar')
                kD = cv2.getTrackbarPos('kD', 'trackbar')
                for i, dist in enumerate(dist_to_motors):
                    dist_percentage = dist / (radius * 2)
                    proportional =  (kP * clamp(1 - dist_percentage, 0, 1) * 255) - 127
                    integral = kI * 0 #TODO
                    if derivative_update_time[i] == None:
                        derivative_update_time[i] = time.perf_counter_ns()
                    if last_dist[i] != None:
                        derivative = kD * (dist_percentage - last_dist[i]) * (derivative_update_time[i] - time.perf_counter_ns()) / 1000000
                        derivative_update_time[i] = time.perf_counter_ns()
                    else:
                        derivative = 0
                    last_dist[i] = dist_percentage
                    motor_output = clamp(proportional + integral + derivative, -127 , 127) * tilt_multiplier + 127 
                    motor_outputs.append(motor_output)
                   
                
                ####THIS NEEDS TO GO HERE SOMEWHERE########
                #activate_motors(ball_direction_degrees,get_edge_elevation(ball_time_to_edge))
                center_motor = int(motor_outputs[0])
                left_motor = int(motor_outputs[2]) #was wrong way around
                right_motor = int(motor_outputs[1]) #was wrong way around
                if print_output:
                    print(f"center:{str(center_motor):<4} left:{str(left_motor):<4} right:{str(right_motor):<4}")
                if serial_output:
                    # Uncomment to make sure platform doesn't move if testing camera
                    # center_motor = 0
                    # left_motor = 0
                    # right_motor = 0
                    # Output to Serial
                    i = 0
                    while i < 3:
                        if ser.in_waiting == 1:
                            serial_recieved = int.from_bytes(ser.read(), byteorder="little")
                            if serial_sent == None or serial_sent == serial_recieved:
                                match serial_count:
                                    case 0:
                                        ser.write(int.to_bytes(center_motor))
                                        serial_sent = center_motor
                                        serial_count = 1
                                        i += 1
                                    case 1:
                                        ser.write(int.to_bytes(left_motor))
                                        serial_sent = left_motor
                                        serial_count = 2
                                        i += 1
                                    case 2:
                                        ser.write(int.to_bytes(right_motor))
                                        serial_sent = right_motor
                                        serial_count = 0
                                        i += 1
                                    case _:
                                        print("ERROR: serial_count out of bounds")
                                        # TODO: Error here
                                print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")
                                serial_recieved = None
                        elif ser.in_waiting > 1:
                            print(f"{ser.read(ser.in_waiting)}")


    cv2.imshow('Frame', frame)
    
    
    key = cv2.waitKey(1) & 0xFF
    # Q to quit
    if key == ord('q'):
        break
    # R to reset disc position whilst in normal mode (not calibration)
    # Doesn't work in manual mode (nothing to calibrate as disc positon/radius set manually)
    if key == ord('r'):
        if not manual_mode:
            if print_output:
                print_output = False
            if get_disc == False:
                get_disc = True
    # P to toggle print output of motor force, ball angle and force multiplier
    if key == ord('p'):
        print_output = not print_output
    # S to toggle serial output of motor forces to arduino
    if key == ord('s'):
        serial_output = not serial_output
        if not serial_initialised:
            print("Please restart to connect to arduino and output serial")
    # Calibration mode, will only outline circles and do nothing else
    # When in calibration mode, press c again to start regular mode (start getting the disc and then lock it and get ball position etc)
    # Doesn't work in manual mode (nothing to calibrate as disc positon/radius set manually)
    if key == ord('c'):
        if not manual_mode:
            if get_disc == False:
                get_disc = True
            calibrate_mode = not calibrate_mode
    # Toggle manual mode, set the position and radius of the disc manually
    if key == ord('m'):
        manual_mode = not manual_mode
    
    if cv2.getWindowProperty('Frame', cv2.WND_PROP_VISIBLE) < 1:
        break

    # print(f"center:{str(center_motor):<4} left:{str(left_motor):<4} right:{str(right_motor):<4}")


cap.release()
cv2.destroyAllWindows()
if serial_initialised:
    center_motor = 0
    left_motor = 0
    right_motor = 0
    i = 0
    while i < 3:
        if ser.in_waiting == 1:
            serial_recieved = int.from_bytes(ser.read(), byteorder="little")
            if serial_sent == None or serial_sent == serial_recieved:
                match serial_count:
                    case 0:
                        ser.write(int.to_bytes(center_motor))
                        serial_sent = center_motor
                        serial_count = 1
                        i += 1
                    case 1:
                        ser.write(int.to_bytes(left_motor))
                        serial_sent = left_motor
                        serial_count = 2
                        i += 1
                    case 2:
                        ser.write(int.to_bytes(right_motor))
                        serial_sent = right_motor
                        serial_count = 0
                        i += 1
                    case _:
                        print("ERROR: serial_count out of bounds")
                        # TODO: Error here
                print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")
                serial_recieved = None
        elif ser.in_waiting > 1:
            print(f"{ser.read(ser.in_waiting)}")
    ser.close()
