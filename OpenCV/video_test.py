import cv2
import numpy as np

vid_capture = cv2.VideoCapture(0)

screenWidth = int(vid_capture.get(3))
screenHeight = int(vid_capture.get(4))

if not vid_capture.isOpened():
    print("Error opening webcam!")
else:
    fps = int(vid_capture.get(cv2.CAP_PROP_FPS))
    print("Frame Rate:", fps, "FPS")

    frame_count = vid_capture.get(cv2.CAP_PROP_FRAME_COUNT)
    print("Frame count:", frame_count)

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

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for circle in circles[0, :]:
                    # Draw the outer circle
                    cv2.circle(frame, (circle[0], circle[1]), circle[2], (0, 0, 255), 2)
                    # Draw the center of the circle
                    cv2.circle(frame, (circle[0], circle[1]), 2, (0, 255, 0), 3)

                    # Calculate the distance from the center of the crosshair to the circle
                    center_x, center_y = int(screenWidth / 2), int(screenHeight / 2)
                    circle_x, circle_y = circle[0], circle[1]
                    distance = np.sqrt((center_x - circle_x)**2 + (center_y - circle_y)**2)

                    # Draw a line from the center of the crosshair to the circle
                    cv2.line(frame, (center_x, center_y), (circle_x, circle_y), (255, 0, 0), 2)

            cv2.line(frame, (int(screenWidth/2), int(screenHeight/2 - 10)), (int(screenWidth/2), int(screenHeight/2 + 10)), (0, 255, 0), 2) # crosshair
            cv2.line(frame, (int(screenWidth/2 - 10), int(screenHeight/2)), (int(screenWidth/2 + 10), int(screenHeight/2)), (0, 255, 0), 2) # crosshair

            cv2.imshow('Frame', frame)
            k = cv2.waitKey(20)
            if k == 113:  # 'q' key to exit
                break
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()
