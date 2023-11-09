import cv2
import numpy as np

# Capture video
vid_capture = cv2.VideoCapture(0)

# Set screen size
screenWidth = int(vid_capture.get(3))
screenHeight = int(vid_capture.get(4))

# Validate that video feed is open
if not vid_capture.isOpened():
    print("Error opening webcam!")
else:
    # Get initial frame rate
    fps = int(vid_capture.get(cv2.CAP_PROP_FPS))
    frame_count = vid_capture.get(cv2.CAP_PROP_FRAME_COUNT)
    print("Frame Rate:", fps, "FPS")
    print("Frame count:", frame_count)

    # Keep the video window open until manually closed
    while vid_capture.isOpened():

        ret, frame = vid_capture.read()

        if ret:
            # Convert the frame to grayscale for circle detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (9, 9), 2)

            # Perform Hough Circle Transform
            circles = cv2.HoughCircles(
                gray, 
                cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=0, maxRadius=0
            )

            # Draw circles if blob found
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for circle in circles[0, :]:
                    # Draw the outer circle
                    cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
                    # Draw the center of the circle
                    cv2.circle(frame, (circle[0], circle[1]), 2, (0, 255, 0), 3)

            # Draw crosshair
            cv2.line(frame, (int(screenWidth/2), int(screenHeight/2 - 10)), (int(screenWidth/2), int(screenHeight/2 + 10)), (0, 255, 0), 2) 
            cv2.line(frame, (int(screenWidth/2 - 10), int(screenHeight/2)), (int(screenWidth/2 + 10), int(screenHeight/2)), (0, 255, 0), 2) 

            cv2.imshow('Frame', frame)
            
            k = cv2.waitKey(20)
            if k == 113:  # 'q' key to exit
                break
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()
