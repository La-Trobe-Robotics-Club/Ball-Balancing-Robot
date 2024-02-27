import serial
import cv2
import numpy as np
import platform

#OS info
platform_os = platform.system()

# Serial config
ser = serial.Serial('/dev/ttyUSB0')
center_motor = 0
left_motor = 0
right_motor = 0
serial_count = 0
serial_sent = None
serial_recieved = None
calibrated = False
calibration_input = None
arduino_calibration_output = None
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

# Open CV config
def nothing(x):
    pass

# Function to detect circles based on color
def detect_circles(frame, lower_color, upper_color, min_radius, max_radius):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
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



# Start capturing video
if platform_os == "Windows":
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
else:
    cap = cv2.VideoCapture(0)

ret, frame = cap.read()
cv2.imshow('Frame', frame)
# Create a set of trackbars for  HSV adjustment of center red disk
cv2.createTrackbar('Lower Hue Disk', 'Frame', 0, 179, nothing)
cv2.createTrackbar('Upper Hue Disk', 'Frame', 179, 179, nothing)
cv2.createTrackbar('Lower Saturation Disk', 'Frame', 100, 255, nothing)
cv2.createTrackbar('Upper Saturation Disk', 'Frame', 255, 255, nothing)
cv2.createTrackbar('Lower Value Disk', 'Frame', 100, 255, nothing)
cv2.createTrackbar('Upper Value Disk', 'Frame', 255, 255, nothing)
cv2.createTrackbar('Min Radius Disk', 'Frame', 0, 150, nothing)
cv2.createTrackbar('Max Radius Disk', 'Frame', 0, 150, nothing)

# Create another set of trackbars for HSV adjustment for the ball (yellow by default)
cv2.createTrackbar('Lower Hue Ball', 'Frame', 25, 179, nothing)  # Adjusted for fluorescent yellow
cv2.createTrackbar('Upper Hue Ball', 'Frame', 40, 179, nothing)
cv2.createTrackbar('Lower Saturation Ball', 'Frame', 150, 255, nothing)
cv2.createTrackbar('Upper Saturation Ball', 'Frame', 255, 255, nothing)
cv2.createTrackbar('Lower Value Ball', 'Frame', 150, 255, nothing)
cv2.createTrackbar('Upper Value Ball', 'Frame', 255, 255, nothing)
cv2.createTrackbar('Min Radius Ball', 'Frame', 0, 150, nothing)
cv2.createTrackbar('Max Radius Ball', 'Frame', 0, 150, nothing)


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

get_disc = True
calibrate_mode = False
disc_centers = []
disc_center = (-1, -1)
right_line = ((-1, -1), (-1, -1))
radii = []
radius = 0
DISC_AVG_SAMPLE_SIZE = 40
NUM_MOTORS = 3

force_multiplier = 0
angle = 0
dist = 0

print_output = False
serial_output = False

motor_outputs = [0] * NUM_MOTORS

center_motor = 0
left_motor = 0
right_motor = 0

seg_size = 360 / NUM_MOTORS
segment_angles = np.linspace(0,360,NUM_MOTORS,endpoint=False)
indexes_and_segment_angles = [(i, int(a)) for i, a in enumerate(segment_angles)]
indexes_and_segment_angles.append((0, 360))
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera. Check camera index and connection.")
        break
    if get_disc or calibrate_mode:
        # Get trackbar positions for red detections
        lower_red = np.array([cv2.getTrackbarPos('Lower Hue Disk', 'Frame'), cv2.getTrackbarPos('Lower Saturation Disk', 'Frame'), cv2.getTrackbarPos('Lower Value Disk', 'Frame')])
        upper_red = np.array([cv2.getTrackbarPos('Upper Hue Disk', 'Frame'), cv2.getTrackbarPos('Upper Saturation Disk', 'Frame'), cv2.getTrackbarPos('Upper Value Disk', 'Frame')])
        min_radius_red = cv2.getTrackbarPos('Min Radius Disk', 'Frame')
        max_radius_red = cv2.getTrackbarPos('Max Radius Disk', 'Frame')
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
            
            for a in segment_angles:
                segment_endpoints.append(calculate_line_endpoint(disc_center, a, radius))
            
            get_disc = False
    if not get_disc or calibrate_mode:
        # Draw averaged circle and center dot
        
        # Draw lines representing the segments
        if not calibrate_mode:
            cv2.line(frame, disc_center, segment_endpoints[0], (0, 255, 0), 3) # Green
            cv2.line(frame, disc_center, segment_endpoints[1], (0, 0, 255), 3) # Red
            cv2.line(frame, disc_center, segment_endpoints[2], (213, 255, 0), 3) # Blue-ish
            for endpoint in segment_endpoints[3:]:
                cv2.line(frame, disc_center, endpoint, (0, 0, 0), 3)
            cv2.circle(frame, disc_center, radius, (0, 255, 0), 2)
            cv2.circle(frame, disc_center, 2, (0, 0, 255), 3)
        # Detect yellow circles
        lower_yellow = np.array([cv2.getTrackbarPos('Lower Hue Ball', 'Frame'), cv2.getTrackbarPos('Lower Saturation Ball', 'Frame'), cv2.getTrackbarPos('Lower Value Ball', 'Frame')])
        upper_yellow = np.array([cv2.getTrackbarPos('Upper Hue Ball', 'Frame'), cv2.getTrackbarPos('Upper Saturation Ball', 'Frame'), cv2.getTrackbarPos('Upper Value Ball', 'Frame')])
        min_radius_yellow = cv2.getTrackbarPos('Min Radius Ball', 'Frame')
        max_radius_yellow = cv2.getTrackbarPos('Max Radius Ball', 'Frame')
        ball_circle = detect_circles(frame, lower_yellow, upper_yellow, min_radius_yellow, max_radius_yellow)
        if ball_circle is not None:
            ball_center = (int(ball_circle[0]), int(ball_circle[1]))
            ball_radius = ball_circle[2]
            cv2.circle(frame, ball_center, ball_radius, (0, 255, 255), 2)
            cv2.circle(frame, ball_center, 2, (255, 255, 255), 3)  # Center point
            if print_output or serial_output:
                dist = calculate_distance(disc_center, ball_center)
                force_multiplier = dist / radius
                cv2.line(frame,disc_center, ball_center, (255, 255, 255), 2)
                ball_angle = angle3pt(right_line, ball_center)
                indexes_and_dists = [(index,abs(ball_angle - segment_angle)) for index, segment_angle in indexes_and_segment_angles]
                sort_by_nearest = sorted(indexes_and_dists, key=lambda x: x[1])
                motor_outputs = [0] * NUM_MOTORS
                motor_outputs[sort_by_nearest[0][0]] = (seg_size-sort_by_nearest[1][1])*force_multiplier*(255/seg_size)
                motor_outputs[sort_by_nearest[1][0]] = (seg_size-sort_by_nearest[0][1])*force_multiplier*(255/seg_size)
                if print_output:
                    print(f"Motor outputs: {motor_outputs}, Ball angle: {ball_angle}, Force multiplier: {force_multiplier}")
                if serial_output:
                    # Code for serial handling goes here
                    pass
    cv2.imshow('Frame', frame)
    
    
    key = cv2.waitKey(1) & 0xFF
    # Q to quit
    if key == ord('q'):
        break
    # R to reset whilst in normal mode (not calibration)
    if key == ord('r'):
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
    # Calibration mode, will only outline circles and do nothing else
    if key == ord('c'):
        if get_disc == False:
            get_disc = True
        calibrate_mode = not calibrate_mode
    
    if cv2.getWindowProperty('Frame', cv2.WND_PROP_VISIBLE) < 1:
        break

    print(f"center:{str(center_motor):<4} left:{str(left_motor):<4} right:{str(right_motor):<4}")
    #Uncomment to make sure platform doesn't move if testing camera
    center_motor = 0
    left_motor = 0
    right_motor = 0

    # Output to Serial
    if ser.in_waiting == 1:
        serial_recieved = int.from_bytes(ser.read(), byteorder="little")
        if serial_sent == None or serial_sent == serial_recieved:
            match serial_count:
                case 0:
                    ser.write(int.to_bytes(center_motor))
                    serial_sent = center_motor
                    serial_count = 1
                case 1:
                    ser.write(int.to_bytes(left_motor))
                    serial_sent = left_motor
                    serial_count = 2
                case 2:
                    ser.write(int.to_bytes(right_motor))
                    serial_sent = right_motor
                    serial_count = 0
                case _:
                    print("ERROR: serial_count out of bounds")
                    # TODO: Error here
            print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")
            serial_recieved = None
    elif ser.in_waiting > 1:
        print(f"{ser.read(ser.in_waiting)}")

cap.release()
cv2.destroyAllWindows()
ser.close()