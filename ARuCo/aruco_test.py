import cv2
import cv2.aruco as aruco
import serial
import time
import imutils

# Initialize the camera
vid_capture = cv2.VideoCapture(0)

# Initialize screen width and height
vid_capture.set(3, 640)
vid_capture.set(4, 480)                                  

vid_capture.set(cv2.CAP_PROP_FPS, 90)
screenWidth = int(vid_capture.get(3))
screenHeight = int(vid_capture.get(4))

ser = serial.Serial('/dev/ttyS0', 115200)

min_marker_size = 27
max_marker_size = 440
min_offset = 0
max_offset = 10

marker_size_range = max_marker_size - min_marker_size
offset_range = max_offset - min_offset

if not vid_capture.isOpened():
    print("Error opening webcam!")
else:
    # Define the ArUco dictionary and parameters
    #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters_create()
    
    start_time = time.time()
    start_time_scan = time.time()
    frames = 0

    while vid_capture.isOpened():
        ret, frame = vid_capture.read()
        #frame = imutils.resize(frame, width=320) #resize window to 1/4 of original
        #fps = vid_capture.get(cv2.CAP_PROP_FPS)
        if ret:
            frames += 1
            elapsed_time = time.time() - start_time;
            if elapsed_time >= 1:
                fps = frames * 2.4 / elapsed_time
                frames = 0
                start_time = time.time()
                                
        
            # Detect ArUco markers in the frame
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
            
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            
            elapsed_time_scan = time.time() - start_time_scan;

            if ids is None:
                if elapsed_time_scan >= 2.5:
                    data_to_send = "SCAN\n"
                    ser.write(bytes(data_to_send, 'utf-8'))
                    print("Marker not detected!")
                    start_time_scan = time.time()
            else:
                
                # Initialize variables to keep track of longest top edge
                longest_top_edge_length = 0
                longest_top_edge_index = None
                
                previous_marker_index = None
                
                # Find the marker with ID 69
                for i in range(len(ids)):
                    # Calculate the length of the top edge
                    top_edge_length = cv2.norm(corners[i][0][0] - corners[i][0][1])
                    
                    if top_edge_length > longest_top_edge_length or top_edge_length == longest_top_edge_length:
                        longest_top_edge_length = top_edge_length
                        longest_top_edge_index = i
                        
                previous_marker_index = longest_top_edge_index
                
                if previous_marker_index is not None:
                    
                    i = previous_marker_index
                    print("Marker detected!")
                    # Draw the detected marker
                    aruco.drawDetectedMarkers(frame, corners) 
                    
                    y_offset = int((top_edge_length - min_marker_size) / marker_size_range * offset_range) + min_offset
                    
                    # Calculate the center of the crosshair
                    crosshair_x = int(screenWidth / 2)

                    crosshair_y = int((1*screenHeight/ 4) + y_offset) 
                
                    
                    cv2.line(frame, (crosshair_x, crosshair_y - 10), (crosshair_x, crosshair_y + 10), (0, 255, 0), 2)  # crosshair
                    cv2.line(frame, (crosshair_x - 10, crosshair_y), (crosshair_x + 10, crosshair_y), (0, 255, 0), 2)  # crosshair
                             
                    # Calculate the center of the detected marker
                    marker_center_x = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                    marker_center_y = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)

                    # Draw a line from the crosshair to the marker
                    cv2.line(frame, (crosshair_x, crosshair_y), (marker_center_x, marker_center_y), (255, 0, 0), 2)
                    
                    dist_x = marker_center_x - crosshair_x
                    dist_y = marker_center_y - crosshair_y
                    
                    cv2.putText(frame, f"X: {dist_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"Y: {dist_y}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                    data_to_send = f"{dist_x},{dist_y}\n"
                    ser.write(bytes(data_to_send, 'utf-8'))
                        
                        
            cv2.imshow('Frame', frame)
            # print("FPS: ", fps)
            k = cv2.waitKey(20)
            if k == 113:  # 'q' key to exit
                break 
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()