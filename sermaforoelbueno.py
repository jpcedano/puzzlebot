import cv2
import numpy as np

def detect_colored_circles(frame):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color ranges for green, red, and yellow
    green_lower = np.array([35, 100, 100])
    green_upper = np.array([85, 255, 255])
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])
    yellow_lower = np.array([25, 100, 100])
    yellow_upper = np.array([35, 255, 255])
    
    # Create masks for each color
    mask_green = cv2.inRange(hsv, green_lower, green_upper)
    mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (9, 9), 2)
    
    # Apply Canny edge detection
    edges = cv2.Canny(cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY), 50, 150)
    
    # Detect circles using Hough Circle Transform
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                               param1=50, param2=30, minRadius=10, maxRadius=30)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # Check which color the circle corresponds to
            if mask_green[y, x] > 0:
                color_label = 'Green'
                color = (0, 255, 0)
            elif mask_red[y, x] > 0:
                color_label = 'Red'
                color = (0, 0, 255)
            elif mask_yellow[y, x] > 0:
                color_label = 'Yellow'
                color = (0, 255, 255)
            else:
                continue
            
            # Draw the circle and label on the frame
            cv2.circle(frame, (x, y), r, color, 4)
            cv2.putText(frame, color_label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Only detect one circle at a time
            break
    
    return frame

def main():
    # Open the camera
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Resize frame to 360x270
        frame = cv2.resize(frame, (360, 270))
        
        # Detect colored circles
        result_frame = detect_colored_circles(frame)
        
        # Display the result
        cv2.imshow('Detected Circles', result_frame)
        
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if _name_ == '_main_':
    main()
