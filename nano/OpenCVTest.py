import cv2
import numpy as np

# Min and max radius of circles
MIN_RADIUS = 0
MAX_RADIUS = 0

# Function to do nothing on trackbar movement
def nothing(x):
    pass

# Create a window with trackbars for HSV adjustment
cv2.namedWindow('Trackbars')
cv2.createTrackbar('Lower Hue', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('Upper Hue', 'Trackbars', 179, 179, nothing)
cv2.createTrackbar('Lower Saturation', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('Upper Saturation', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('Lower Value', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('Upper Value', 'Trackbars', 255, 255, nothing)

def detect_red_circles(frame, lower_red, upper_red):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
    
    # Detect circles in the combined mask
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30, minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)
    
    largest_circle = None
    if circles is not None:
        circles = np.uint16(np.around(circles))
        max_radius = 0
        # Find the largest circle
        for i in circles[0, :]:
            if i[2] > max_radius:
                max_radius = i[2]
                largest_circle = i
        
        if largest_circle is not None:
            center = (largest_circle[0], largest_circle[1])  # Center of the largest circle
            radius = largest_circle[2]  # Radius of the largest circle
            # Draw the outer circle
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, center, 2, (0, 0, 255), 3)
            return frame, center

    return frame, None

# Start capturing video from the first camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera. Check camera index and connection.")
        break

    # Get current positions of the trackbars
    lh = cv2.getTrackbarPos('Lower Hue', 'Trackbars')
    uh = cv2.getTrackbarPos('Upper Hue', 'Trackbars')
    ls = cv2.getTrackbarPos('Lower Saturation', 'Trackbars')
    us = cv2.getTrackbarPos('Upper Saturation', 'Trackbars')
    lv = cv2.getTrackbarPos('Lower Value', 'Trackbars')
    uv = cv2.getTrackbarPos('Upper Value', 'Trackbars')

    # Define the HSV range for red color from the trackbars
    lower_red = np.array([lh, ls, lv])
    upper_red = np.array([uh, us, uv])

    # Process the frame
    processed_frame, center = detect_red_circles(frame, lower_red, upper_red)
    cv2.imshow('Frame', processed_frame)
    
    if center:
        print(f"Circle's center: {center}")
    
    # Break loop and close window with 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # Destroy window if close window button is pressed
    if cv2.getWindowProperty('Frame', cv2.WND_PROP_VISIBLE) < 1:
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
#low hue: 80
#high hue: 179
#low sat: 66
#high sat: 165


### include a slider for radius range. Use this once webcam is mounted to limit circle radius to appropriate range.