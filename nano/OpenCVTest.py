import cv2
import numpy as np

def nothing(x):
    pass

# Create windows with trackbars for HSV adjustment for the red disc
cv2.namedWindow('Trackbars Red')
cv2.createTrackbar('Lower Hue', 'Trackbars Red', 0, 179, nothing)
cv2.createTrackbar('Upper Hue', 'Trackbars Red', 179, 179, nothing)
cv2.createTrackbar('Lower Saturation', 'Trackbars Red', 100, 255, nothing)
cv2.createTrackbar('Upper Saturation', 'Trackbars Red', 255, 255, nothing)
cv2.createTrackbar('Lower Value', 'Trackbars Red', 100, 255, nothing)
cv2.createTrackbar('Upper Value', 'Trackbars Red', 255, 255, nothing)
cv2.createTrackbar('Min Radius', 'Trackbars Red', 0, 150, nothing)
cv2.createTrackbar('Max Radius', 'Trackbars Red', 0, 150, nothing)

# Create another set of trackbars for HSV adjustment for the yellow ball
cv2.namedWindow('Trackbars Yellow')
cv2.createTrackbar('Lower Hue', 'Trackbars Yellow', 25, 179, nothing)  # Adjusted for fluorescent yellow
cv2.createTrackbar('Upper Hue', 'Trackbars Yellow', 40, 179, nothing)
cv2.createTrackbar('Lower Saturation', 'Trackbars Yellow', 150, 255, nothing)
cv2.createTrackbar('Upper Saturation', 'Trackbars Yellow', 255, 255, nothing)
cv2.createTrackbar('Lower Value', 'Trackbars Yellow', 150, 255, nothing)
cv2.createTrackbar('Upper Value', 'Trackbars Yellow', 255, 255, nothing)
cv2.createTrackbar('Min Radius', 'Trackbars Yellow', 0, 150, nothing)
cv2.createTrackbar('Max Radius', 'Trackbars Yellow', 0, 150, nothing)

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

# Start capturing video
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera. Check camera index and connection.")
        break

    # Get trackbar positions for both red and yellow detections
    lower_red = np.array([cv2.getTrackbarPos('Lower Hue', 'Trackbars Red'), cv2.getTrackbarPos('Lower Saturation', 'Trackbars Red'), cv2.getTrackbarPos('Lower Value', 'Trackbars Red')])
    upper_red = np.array([cv2.getTrackbarPos('Upper Hue', 'Trackbars Red'), cv2.getTrackbarPos('Upper Saturation', 'Trackbars Red'), cv2.getTrackbarPos('Upper Value', 'Trackbars Red')])
    min_radius_red = cv2.getTrackbarPos('Min Radius', 'Trackbars Red')
    max_radius_red = cv2.getTrackbarPos('Max Radius', 'Trackbars Red')

    lower_yellow = np.array([cv2.getTrackbarPos('Lower Hue', 'Trackbars Yellow'), cv2.getTrackbarPos('Lower Saturation', 'Trackbars Yellow'), cv2.getTrackbarPos('Lower Value', 'Trackbars Yellow')])
    upper_yellow = np.array([cv2.getTrackbarPos('Upper Hue', 'Trackbars Yellow'), cv2.getTrackbarPos('Upper Saturation', 'Trackbars Yellow'), cv2.getTrackbarPos('Upper Value', 'Trackbars Yellow')])
    min_radius_yellow = cv2.getTrackbarPos('Min Radius', 'Trackbars Yellow')
    max_radius_yellow = cv2.getTrackbarPos('Max Radius', 'Trackbars Yellow')

    # Detect red circles
    red_circle = detect_circles(frame, lower_red, upper_red, min_radius_red, max_radius_red)
    if red_circle is not None:
        cv2.circle(frame, (red_circle[0], red_circle[1]), red_circle[2], (0, 255, 0), 2)
        cv2.circle(frame, (red_circle[0], red_circle[1]), 2, (0, 0, 255), 3)  # Center point

    # Detect yellow circles
    yellow_circle = detect_circles(frame, lower_yellow, upper_yellow, min_radius_yellow, max_radius_yellow)
    if yellow_circle is not None:
        cv2.circle(frame, (yellow_circle[0], yellow_circle[1]), yellow_circle[2], (0, 255, 255), 2)
        cv2.circle(frame, (yellow_circle[0], yellow_circle[1]), 2, (255, 0, 0), 3)  # Center point

    cv2.imshow('Frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.getWindowProperty('Frame', cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()
