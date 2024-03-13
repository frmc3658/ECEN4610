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
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            lower_red1 = np.array([00, 200, 200])
            upper_red1 = np.array([50, 255, 255])
            lower_red2 = np.array([120, 156,156])
            upper_red2 = np.array([180, 255, 255])
            
            #lower_green = np.array([36, 25, 25])
            #upper_green = np.array([86, 255, 255])
            
            # Convert the frame to grayscale for circle detection
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            #green_mask = cv2.inRange(hsv, lower_green, upper_green)
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest = max(contours, key=cv2.contourArea)
                
                cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
#             # Perform Hough Circle Transform
#             params = cv2.SimpleBlobDetector_Params()
#             
#             params.filterByArea = True
#             params.minArea = 100000
#             
#             detector = cv2.SimpleBlobDetector_create(params)
#             
#             red_keypoints = detector.detect(red_mask)
            #green_keypoints = detector.detect(green_mask)
            
#             red_blob_image = cv2.drawKeypoints(frame, red_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#             #green_blob_image = cv2.drawKeypoints(frame, red_keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#             if red_blob_image is not None:
#                 red_blob_image = np.uint16(np.around(red_blob_image))
# 
#                 for red_blob_image in red_blob_image[0, :]:
#                     # Draw the outer circle
#                     cv2.circle(frame, (red_blob_image[0],red_blob_image[1]) ,red_blob_image[2], (0, 0, 255), 2)
#                     # Draw the center of the circle
# #                     cv2.circle(frame, (red_blob_image[0], red_blob_image[1]), 2, (0, 255, 0), 3)


            # Draw crosshair
            cv2.line(frame, (int(screenWidth/2), int(screenHeight/2 - 10)), (int(screenWidth/2), int(screenHeight/2 + 10)), (0, 255, 0), 2) 
            cv2.line(frame, (int(screenWidth/2 - 10), int(screenHeight/2)), (int(screenWidth/2 + 10), int(screenHeight/2)), (0, 255, 0), 2) 
#             cv2.imshow("red", red_blob_image)
#             cv2.imshow("green", green_blob_image)
            
            cv2.imshow("Frame", frame)
            k = cv2.waitKey(30)
            if k == 113:  # 'q' key to exit
                break
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()
