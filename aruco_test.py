import cv2
import cv2.aruco as aruco

# Initialize the camera
vid_capture = cv2.VideoCapture(0)
screenWidth = int(vid_capture.get(3))
screenHeight = int(vid_capture.get(4))

if not vid_capture.isOpened():
    print("Error opening webcam!")
else:
    # Define the ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    aruco_params = aruco.DetectorParameters_create()

    while vid_capture.isOpened():
        ret, frame = vid_capture.read()
        # frame = cv2.flip(frame, 1)
        if ret:
            # Calculate the center of the crosshair
            crosshair_x, crosshair_y = int(screenWidth / 2), int(screenHeight / 2)

            cv2.line(frame, (crosshair_x, crosshair_y - 10), (crosshair_x, crosshair_y + 10), (0, 255, 0), 2)  # crosshair
            cv2.line(frame, (crosshair_x - 10, crosshair_y), (crosshair_x + 10, crosshair_y), (0, 255, 0), 2)  # crosshair

            # Detect ArUco markers in the frame
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            if ids is not None:
                # Find the marker with ID 69
                for i in range(len(ids)):
                    if ids[i] == 69:
                        # Draw the detected marker
                        aruco.drawDetectedMarkers(frame, corners)

                        # Calculate the center of the detected marker
                        marker_center_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                        marker_center_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)

                        # Draw a line from the crosshair to the marker
                        cv2.line(frame, (crosshair_x, crosshair_y), (marker_center_x, marker_center_y), (255, 0, 0), 2)
            frame = cv2.flip(frame, 1)
            cv2.imshow('Frame', frame)
            k = cv2.waitKey(20)
            if k == 113:  # 'q' key to exit
                break
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()
