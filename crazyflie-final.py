# Code adapted from: https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/autonomousSequence.py

import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

# CrazyFlie imports:

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

group_number = 2

# Possibly try 0, 1, 2 ...
camera = 1

# """
# RED DETECTION
# """

# cap = cv2.VideoCapture(camera)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()
    
#     # These define the upper and lower HSV for the red obstacles.
#     # Note that the red color wraps around 180, so there are two intervals.
#     # Tuning of these values will vary depending on the camera.
    
# #     # RED
# #     lb1 = (145, 35, 75)
# #     ub1 = (180, 255, 255)
# #     lb2 = (0, 75, 75)
# #     ub2 = (20, 255, 255)

# #     # Perform contour detection on the input frame.
# #     hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# #     hsv2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# #     # Compute mask of red obstacles in either color range.
# #     mask1 = cv2.inRange(hsv1, lb1, ub1)
# #     mask2 = cv2.inRange(hsv2, lb2, ub2)
# #     # Combine the masks.
# #     mask = cv2.bitwise_or(mask1, mask2)
   
#     # Compute
#     cv2.imshow('mask', frame)    

#     # Hit q to quit.
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release the capture
# cap.release()
# cv2.destroyAllWindows()

# """
# BLUE DETECTION
# """

# cap = cv2.VideoCapture(camera)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()
    
#     # These define the upper and lower HSV for the red obstacles.
#     # Note that the red color wraps around 180, so there are two intervals.
#     # Tuning of these values will vary depending on the camera.
    
#     # BLUE
# #     lb1 = (110, 50, 50)
# #     ub1 = (130, 255, 255)
    

# #     # Perform contour detection on the input frame.
# #     hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# #     # Compute mask of red obstacles in either color range.
# #     mask = cv2.inRange(hsv1, lb1, ub1)
    
#     # Compute
#     cv2.imshow('mask', frame)    

#     # Hit q to quit.
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release the capture
# cap.release()
# cv2.destroyAllWindows()

HEIGHT = 0.75

# Get the current crazyflie position:
def position_estimate(scf):
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    
    with SyncLogger(scf, log_config) as logger:
        print("-------------")
        i = 0
        for log_entry in logger:
            if i == 5:
                break
            data = log_entry[1]
            x = data['kalman.varPX']
            y = data['kalman.varPY']
            z = data['kalman.varPZ']
            print("DATA: ", x, y, z)
            i += 1
            
#     print("POSITION: ", x, y, z)
    return x, y, z


# Set the built-in PID controller:
def set_PID_controller(cf):
    # Set the PID Controller:
    print('Initializing PID Controller')
    cf.param.set_value('stabilizer.controller', '1')
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.5)
#     time.sleep(2)
    return


# Ascend and hover at 1m:
def ascend_and_hover(cf):
    # Ascend:
    print("Ascending")
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT * y / 10)
        time.sleep(0.1)
    # Hover at 0.5 meters:
    print("Hovering")
    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, HEIGHT)
        time.sleep(0.1)
    return


# Sort through contours in the image
def findGreatesContour(contours):
    largest_area = 0
    largest_contour_index = -1
    i = 0
    total_contours = len(contours)

    while i < total_contours:
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            largest_area = area
            largest_contour_index = i
        i += 1

    #print(largest_area)

    return largest_area, largest_contour_index


# Find contours in the image
def check_contours(frame):

#     print('Checking image:')

    # These define the upper and lower HSV for the red obstacles.
    # Note that the red color wraps around 180, so there are two intervals.
    # Tuning of these values will vary depending on the camera.
    lb1 = (145, 35, 75)
    ub1 = (180, 255, 255)
    lb2 = (0, 75, 75)
    ub2 = (20, 255, 255)

    # Perform contour detection on the input frame.
    hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Compute mask of red obstacles in either color range.
    mask1 = cv2.inRange(hsv1, lb1, ub1)
    mask2 = cv2.inRange(hsv2, lb2, ub2)
    # Combine the masks.
    mask = cv2.bitwise_or(mask1, mask2)

    # Use the OpenCV findContours function.
    # Note that there are three outputs, but we discard the first one.
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    largest_area, largest_contour_index = findGreatesContour(contours)

#     print(largest_area)
    
    return largest_area

#     if largest_area > 35000:
#         return True, largest_area
#     else:
#         return False, largest_area

def adjust_position(cf, current_x, current_y, current_yaw, sleep=0.5):

    position = [current_x, current_y, HEIGHT, current_yaw]
    print('Setting position {}'.format(position))
    for i in range(5):
        cf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])
        time.sleep(0.1)
    time.sleep(sleep)

# Hover, descend, and stop all motion:
def hover_and_descend(cf):
    print('Descending:')
    # Hover at 0.5 meters:
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.5)
        time.sleep(0.1)
    # Descend:
    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        time.sleep(0.1)
    # Stop all motion:
    for i in range(10):
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)
    return

ABOVE_THRESHOLD = 8000
BELOW_THRESHOLD = 5000
frames = []

def check_contours_stable(cap, crop):
    contour_areas = []
    local_frames = []
    
    for i in range(7):
        ret = False
        while not ret:
            ret, frame = cap.read()
        
        width = len(frame[0])
        height = len(frame)
        frame = frame[height//3:2*height//3, int(width/crop):int((crop-1)*width/crop)]
        largest_area = check_contours(frame)
        
        frames.append(frame)
        local_frames.append(frame)
        
        cv2.imwrite(f"frames_{len(frames)}.jpg", frame)
        print("getting frame", len(frames))
        
        contour_areas.append(largest_area)
        
    contour_areas.sort()
    print("Contour area: ", contour_areas)
    
    return contour_areas, local_frames

def check_contours_above(cap, crop=3.5):
    contour_areas, local_frames = check_contours_stable(cap, crop)
    
    above_threshold = contour_areas[3] > ABOVE_THRESHOLD
    
    if above_threshold:
        for frame in local_frames:
            cv2.imwrite(f"big_contours/frames_{len(frames)}.jpg", frame)
    
    return above_threshold, contour_areas[3]

def check_contours_below(cap, crop=3.5):
    contour_areas, local_frames = check_contours_stable(cap, crop)
    
    below_threshold = contour_areas[3] < BELOW_THRESHOLD
    
    if below_threshold:
        for frame in local_frames:
            cv2.imwrite(f"small_contours/frames_{len(frames)}.jpg", frame)
    
    return below_threshold, contour_areas[3]

# Set the URI the Crazyflie will connect to
uri = f'radio://0/{group_number}/2M'

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Scan for Crazyflies in range of the antenna:
print('Scanning interfaces for Crazyflies...')
available = cflib.crtp.scan_interfaces()

# List local CrazyFlie devices:
print('Crazyflies found:')
for i in available:
    print(i[0])

    
# BOUNDS_WIDTH = 150, actually 160 cm for Andlinger
BOUNDS_WIDTH = 0.7 # actually 150, with 20 cm buffer on each side
LEFT, RIGHT = 1, -1
DIR = LEFT

# Check that CrazyFlie devices are available:
if len(available) == 0:
    print('No Crazyflies found, cannot run example')
else:
    ## Ascent to hover; run the sequence; then descend from hover:
    # Use the CrazyFlie corresponding to team number:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        set_PID_controller(cf)
                
        # Get the Crazyflie class instance:
        with MotionCommander(scf, default_height=0.5) as mc:
            current_y = 0.0
            current_x = 0.0
            angle = 0 # positive angle is turning left
            

            # Initialize and ascend:
            t = time.time()
            elapsed = time.time() - t
            ascended_bool = 0

            cap = cv2.VideoCapture(camera)
            while(cap.isOpened()):

                ret, frame = cap.read()

                elapsed = time.time() - t

                if(elapsed > 5.0):

                    print('Capturing.....')

                    if ret:
                        #cv2.imshow('frame',frame)

                        if(ascended_bool==0):
#                             set_PID_controller(cf)
#                             ascend_and_hover(cf)
                            ascended_bool = 1
                        else:
                            print("estimating position...")
                            position_estimate(scf)
                            forward_dist = 0.1
#                             lateral_movement = 0.25
                    
                            red_above_threshold, _ = check_contours_above(cap)
                            if red_above_threshold:
                    
                                if current_y <= 0.0: # we're on the right
                                    print("Turning left...")
                                    while angle < 90:
                                        mc.turn_left(10, 10)
                                        angle += 10
                                        print(f"Current angle: {angle}")

                                        below_threshold, _ = check_contours_below(cap, 3)

                                        if below_threshold:
                                            break
                                else: # we're on the left
                                    print("Turning right...")
                                    while angle > -90:
                                        mc.turn_right(10, 10)
                                        angle -= 10
                                        print(f"Current angle: {angle}")

                                        below_threshold, _ = check_contours_below(cap, 3)

                                        if below_threshold:
                                            break
                                
                            rad_angle = np.radians(angle)
                            projected_x = current_x + forward_dist*np.cos(rad_angle)
                            projected_y = current_y + forward_dist*np.sin(rad_angle)
                            
                            if projected_y < -BOUNDS_WIDTH/2 or projected_y > BOUNDS_WIDTH/2:
                                print("OUT OF BOUNDS")
                                
                                if projected_y > 0: # we're on the left
                                    print("Turning right...")
                                    while angle > -10: 
                                        # turn right
                                        mc.turn_right(10, 10)
                                        angle -= 10
                                        print(f"Current angle: {angle}")
                                else:
                                    print("Turning left...")
                                    while angle < 10: # we're on the right
                                        # turn left
                                        mc.turn_left(10, 10)
                                        angle += 10
                                        print(f"Current angle: {angle}")
                                
                                print(f"Angle, x, y: {angle}, {current_x}, {current_y}")
#                                 actual_x, actual_y, _ = cf.commander.get_position()
#                                 print(f"Actual x, y: {actual_x}, {actual_y}")
                                
                                continue
                            
                            current_x = projected_x
                            current_y = projected_y
                            
                            mc.forward(forward_dist, 0.1)
                            print(f"Angle, x, y: {angle}, {current_x}, {current_y}")
#                             actual_x, actual_y, _ = cf.commander.get_position()
#                             print(f"Actual x, y: {actual_x}, {actual_y}")
                            
                            time.sleep(0.5)
                                
                if(elapsed > 120.0):
                    break

            cap.release()

            # Descend and stop all motion:
            hover_and_descend(cf)
            
            mc.stop()

print('Done!')