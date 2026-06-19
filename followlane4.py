#!/usr/bin/env python3
import cv2
import numpy as np
from gpiozero import PWMOutputDevice
import time

# ============================================================
#    HARDWARE SETUP
# ============================================================
left_f  = PWMOutputDevice(12, frequency=100)
left_b  = PWMOutputDevice(16, frequency=100)
right_f = PWMOutputDevice(20, frequency=100)
right_b = PWMOutputDevice(21, frequency=100)

# ============================================================
#    HELPER FUNCTIONS
# ============================================================
def clamp(x, lo=0, hi=100):
    return max(lo, min(hi, int(x)))

def set_motor(left, right):
    actual_left = right
    actual_right = left

    l_val = clamp(abs(actual_left)) / 100.0
    r_val = clamp(abs(actual_right)) / 100.0
    
    if actual_left >= 0:
        left_f.value = l_val; left_b.value = 0
    else:
        left_f.value = 0; left_b.value = l_val
    if actual_right >= 0:
        right_f.value = r_val; right_b.value = 0
    else:
        right_f.value = 0; right_b.value = r_val

def stop_motor():
    left_f.value = 0; left_b.value = 0
    right_f.value = 0; right_b.value = 0

def has_red_lane(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                          cv2.inRange(hsv, lower_red2, upper_red2))
    roi_mask = mask[FRAME_HEIGHT//2:, :]
    M = cv2.moments(roi_mask)
    return M["m00"] > 500

# --- THE 3-PHASE ACKERMANN TURN ---
def turn_until_red(direction, cap):
    # PHASE 1: Drive Straight for 0.3 seconds to center in intersection
    set_motor(BASE_SPEED, BASE_SPEED)
    time.sleep(0.3)
    
    # PHASE 2: Smooth Ackermann Arc
    OUTER_SPEED = 60
    INNER_SPEED = -40  # Slower inner wheel (creates the tight arc)
    
    print(f"Executing smooth {direction} Ackermann Arc...")

    if direction == "LEFT":
        set_motor(INNER_SPEED, OUTER_SPEED)
    else:
        set_motor(OUTER_SPEED, INNER_SPEED)

    # Duration for the arc
    time.sleep(0.6)

    # PHASE 3: Search Phase (Pivot only if the arc missed the line)
    print("Searching for red lane...")
    search_start = time.time()
    PIVOT_SPEED = 30
    
    while True:
        ret, frame = cap.read()
        if not ret: continue

        if has_red_lane(frame):
            break

        if time.time() - search_start > 20.0:
            break

        # Only pivot if we are truly lost
        if direction == "LEFT":
            set_motor(-PIVOT_SPEED, PIVOT_SPEED)
        else:
            set_motor(PIVOT_SPEED, -PIVOT_SPEED)

        time.sleep(0.01)

    stop_motor()
    time.sleep(0.5)

# ============================================================
#    VISION PARAMS & INITIALIZATION
# ============================================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BASE_SPEED   = 40
MIN_SPEED    = 15
STEER_GAIN   = 0.50  
STEER_D      = 0.60  

APPROACH_SPEED     = 18    
ARUCO_TRIGGER_AREA = 1500  

# HSV Red Ranges
lower_red1 = np.array([0, 110, 70])
upper_red1 = np.array([8, 255, 255])
lower_red2 = np.array([165, 110, 70])
upper_red2 = np.array([180, 255, 255])

vertical_kernel = np.ones((25, 5), np.uint8)

# ArUco Setup
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
    OLD_ARUCO = False
except AttributeError:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
    OLD_ARUCO = True

TAG_MAP = {1: "STRAIGHT", 2: "STRAIGHT", 3: "RIGHT", 4: "RIGHT", 5: "LEFT"}

# Variables for Memory and Smoothing
last_turn_time = 0
TURN_COOLDOWN = 1.0  
last_error = 0       
ema_error = 0        
ERROR_ALPHA = 0.4    
ema_speed = 0        
SPEED_ALPHA = 0.2    
last_steer_value = 0 # Initialization for the gap-coasting memory

# ============================================================
#    MAIN LOOP
# ============================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
time.sleep(0.5)

print("Robot Online. Non-Stop Line Following + Ackermann Turns Active!")

try:
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # --- 1. ARUCO DETECTION ---
        action_to_execute = None
        approaching_junction = False
        
        if (time.time() - last_turn_time) > TURN_COOLDOWN:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if OLD_ARUCO:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
            else:
                corners, ids, _ = aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                tid = ids[0][0]
                if tid in TAG_MAP:
                    marker_corners = corners[0][0]
                    area = cv2.contourArea(marker_corners)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    
                    if area >= ARUCO_TRIGGER_AREA:
                        action_to_execute = TAG_MAP[tid]
                    else:
                        approaching_junction = True

        # --- 2. EXECUTE ARUCO ACTION ---
        if action_to_execute is not None:
            # NO STOPPING: The 0.5s time.sleep and stop_motor have been removed.
            # It transitions instantly from driving to executing the action!
            
            if action_to_execute == "STRAIGHT":
                set_motor(BASE_SPEED, BASE_SPEED)
                time.sleep(1.0)
            elif action_to_execute == "LEFT":
                turn_until_red("LEFT", cap)
            elif action_to_execute == "RIGHT":
                turn_until_red("RIGHT", cap)
            
            last_turn_time = time.time()
            ema_speed = 0
            continue 

        # --- 3. RED LINE FOLLOWING ---
        else:
            current_base_speed = APPROACH_SPEED if approaching_junction else BASE_SPEED
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            raw_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                                      cv2.inRange(hsv, lower_red2, upper_red2))
            
            clean_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, vertical_kernel)
            
            steer_slice = clean_mask[80:240, :]
            M = cv2.moments(steer_slice)

            if M["m00"] > 100:
                cX = int(M["m10"] / M["m00"])
                cY = 160 
                
                raw_error = cX - (FRAME_WIDTH // 2)
                ema_error = (ERROR_ALPHA * raw_error) + ((1.0 - ERROR_ALPHA) * ema_error)
                derivative = ema_error - last_error
                last_error = ema_error
                
                steer = (STEER_GAIN * ema_error) + (STEER_D * derivative)
                
                # Save the steer value right before executing it, so if the line is lost 
                # on the next frame, the robot remembers its trajectory!
                last_steer_value = steer 
                
                target_speed = max(MIN_SPEED, current_base_speed - abs(steer) * 0.8)
                
                if ema_speed == 0: ema_speed = target_speed 
                ema_speed = (SPEED_ALPHA * target_speed) + ((1.0 - SPEED_ALPHA) * ema_speed)
                
                # Pure Ackermann math (never backward on standard track)
                set_motor(max(0, ema_speed + steer), max(0, ema_speed - steer))
                cv2.circle(frame, (cX, cY), 8, (0, 255, 0), -1)
                
            else:
                # GAP HANDLING: Never Stop. 
                # Keep moving at BASE_SPEED and use the last recorded steering curve
                print("Line lost, maintaining trajectory...")
                set_motor(BASE_SPEED + last_steer_value, BASE_SPEED - last_steer_value)

except KeyboardInterrupt:
    print("\nUser Stopped Program.")
finally:
    stop_motor()
    if 'cap' in locals() and cap is not None: cap.release()