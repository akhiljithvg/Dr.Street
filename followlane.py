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

def turn_until_red(direction, cap):
    TURN_SPEED = 35
    print(f"Executing slow {direction} turn for 0.8s, then searching for red lane...")

    if direction == "LEFT":
        set_motor(-TURN_SPEED, TURN_SPEED)
    else:
        set_motor(TURN_SPEED, -TURN_SPEED)

    end_time = time.time() + 1.0
    while time.time() < end_time:
        ret, frame = cap.read()
        if not ret: continue
        time.sleep(0.01)

    stop_motor()
    time.sleep(0.05)

    search_start = time.time()
    while True:
        ret, frame = cap.read()
        if not ret: continue

        if has_red_lane(frame):
            print("Red lane detected, stopping turn.")
            break

        if time.time() - search_start > 5.0:
            print("Search timeout; stopping anyway.")
            break

        if direction == "LEFT":
            set_motor(-TURN_SPEED, TURN_SPEED)
        else:
            set_motor(TURN_SPEED, -TURN_SPEED)

        time.sleep(0.01)

    stop_motor()
    time.sleep(0.5)

# ============================================================
#    VISION PARAMS & INITIALIZATION
# ============================================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

BASE_SPEED   = 30
MIN_SPEED    = 15
STEER_GAIN   = 0.50  # Slightly increased to make sure it pulls into the curve
STEER_D      = 0.60  

APPROACH_SPEED     = 14    
ARUCO_TRIGGER_AREA = 1500  

# HSV Red Ranges
lower_red1 = np.array([0, 110, 70])
upper_red1 = np.array([8, 255, 255])
lower_red2 = np.array([165, 110, 70])
upper_red2 = np.array([180, 255, 255])

# Vertical kernel to bridge the gaps in dashed lines
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

last_turn_time = 0
TURN_COOLDOWN = 1.0  
last_error = 0       
ema_error = 0        
ERROR_ALPHA = 0.4    
ema_speed = 0        
SPEED_ALPHA = 0.2    

# NEW: Trajectory Memory Variables
last_steer_value = 0
frames_lost = 0

# ============================================================
#    MAIN LOOP
# ============================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
time.sleep(0.5)

print("Robot Online. Tracking Red Dashes + Advanced Trajectory Memory...")

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
                        print(f"Target Area Reached ({area:.0f} px). Triggering action!")
                    else:
                        approaching_junction = True
                        cv2.putText(frame, f"APPROACHING: {area:.0f}px", (20, 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- 2. EXECUTE ARUCO ACTION ---
        if action_to_execute is not None:
            set_motor(BASE_SPEED, BASE_SPEED)
            time.sleep(0.5)
            
            if action_to_execute == "STRAIGHT":
                pass 
            elif action_to_execute == "LEFT":
                turn_until_red("LEFT", cap)
            elif action_to_execute == "RIGHT":
                turn_until_red("RIGHT", cap)
            
            last_turn_time = time.time()
            last_error = 0
            ema_error = 0
            ema_speed = 0
            last_steer_value = 0
            continue 

        # --- 3. RED LINE FOLLOWING ---
        else:
            current_base_speed = APPROACH_SPEED if approaching_junction else BASE_SPEED
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            raw_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                                      cv2.inRange(hsv, lower_red2, upper_red2))
            
            clean_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, vertical_kernel)
            
            # THE FIX: Expanded the box slightly upwards to catch the curve earlier, 
            # while still keeping it low enough to prevent corner-cutting.
            steer_slice = clean_mask[80:300, :]
            M = cv2.moments(steer_slice)
            
            cv2.rectangle(frame, (0, 100), (FRAME_WIDTH, 200), (255, 255, 0), 1)

            if M["m00"] > 100:
                frames_lost = 0 # We see the line, reset the lost counter
                
                cX = int(M["m10"] / M["m00"])
                cY = 150 
                
                raw_error = cX - (FRAME_WIDTH // 2)
                
                ema_error = (ERROR_ALPHA * raw_error) + ((1.0 - ERROR_ALPHA) * ema_error)
                derivative = ema_error - last_error
                last_error = ema_error
                
                steer = (STEER_GAIN * ema_error) + (STEER_D * derivative)
                last_steer_value = steer # Save this in case we lose the line mid-curve!
                
                target_speed = max(MIN_SPEED, current_base_speed - abs(steer) * 0.6)
                
                if ema_speed == 0: ema_speed = target_speed 
                ema_speed = (SPEED_ALPHA * target_speed) + ((1.0 - SPEED_ALPHA) * ema_speed)
                
                set_motor(ema_speed + steer, ema_speed - steer)
                cv2.circle(frame, (cX, cY), 8, (0, 255, 0), -1)
                
            else:
                frames_lost += 1
                
                # TRAJECTORY MEMORY: If we lose the line, don't just drive straight!
                # If we were steering hard right before we lost the line, keep steering hard!
                if frames_lost < 15: # For roughly 0.5 seconds of losing the line...
                    # Coast at MIN_SPEED and apply the last known steering value
                    set_motor(MIN_SPEED + last_steer_value, MIN_SPEED - last_steer_value)
                    cv2.putText(frame, "CURVE MEMORY ACTIVE", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
                else:
                    # If it's been lost for a long time, it's a true gap, coast straight smoothly
                    target_speed = MIN_SPEED
                    ema_speed = (SPEED_ALPHA * target_speed) + ((1.0 - SPEED_ALPHA) * ema_speed)
                    set_motor(ema_speed, ema_speed)
                    cv2.putText(frame, "GAP DETECTED (COASTING)", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        #cv2.imshow("Robot View", frame)
        #if cv2.waitKey(1) == ord('q'): break

except KeyboardInterrupt:
    print("\nUser Stopped Program.")
finally:
    print("Stopping motors and cleaning up...")
    stop_motor()
    time.sleep(0.2) 
    try:
        left_f.close(); left_b.close(); right_f.close(); right_b.close()
    except Exception as e:
        print(f"Error closing motor pins: {e}")
    if 'cap' in locals() and cap is not None: cap.release()