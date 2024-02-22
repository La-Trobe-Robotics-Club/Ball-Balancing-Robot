import cv2
import numpy as np

def detect_red_circles(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Estimated HSV ranges for your specific red, adjusted for lighting variability
    # Note: These ranges are approximations; you may need to fine-tune them
    lower_red = np.array([0, 100, 100])  # Adjusted based on provided RGB values
    upper_red = np.array([10, 255, 255])
    
    # Second range to capture the darker shade of red
    lower_red2 = np.array([160, 100, 100])  # Higher hue for dark red
    upper_red2 = np.array([179, 255, 255])
    
    # Create masks for red colors
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    # Combine masks
    mask = mask1 + mask2
    mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
    
    # Detect circles in the combined mask
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30, minRadius=0, maxRadius=0)
    
    center = None
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])  # Center of the circle
            radius = i[2]  # Radius of the circle
            # Draw the outer circle
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            # Draw the center of the circle
            cv2.circle(frame, center, 2, (0, 0, 255), 3)
    
    return frame, center

# Start capturing video from camera 2
cap = cv2.VideoCapture(1)

while True:
    _, frame = cap.read()
    processed_frame, center = detect_red_circles(frame)
    cv2.imshow('Frame', processed_frame)
    
    if center:
        print(f"Circle's center: {center}")
    
    # Break loop and close window with 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
