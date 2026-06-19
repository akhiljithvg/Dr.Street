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
    # Swapping left and right values to correct physical wiring logic
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
    TURN_SPEED = 40
    print(f"Executing slow {direction} turn for 0.3s, then searching for red lane...")

    if direction == "LEFT":
        set_motor(-TURN_SPEED, TURN_SPEED)
    else:
        set_motor(TURN_SPEED, -TURN_SPEED)

    end_time = time.time() + 0.8
    while time.time() < end_time:
        ret, frame = cap.read()
        if not ret:
            continue
        time.sleep(0.01)

    stop_motor()
    time.sleep(0.05)

    search_start = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

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
#    VISION PARAMS
# ============================================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
BASE_SPEED   = 20    # Increased for more torque
STEER_GAIN   = 0.50  # P-gain (reduced to reduce wobble)
STEER_D      = 0.50  # D-gain (dampens oscillation/fishtailing)

# HSV Red Ranges
lower_red1 = np.array([0, 110, 70])
upper_red1 = np.array([8, 255, 255])
lower_red2 = np.array([165, 110, 70])
upper_red2 = np.array([180, 255, 255])

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
TURN_COOLDOWN = 1.0  # Seconds to ignore tags after a turn
last_error = 0       # Used for smoothing steering (PD Controller)
ema_error = 0        # Exponential Moving Average for error
ERROR_ALPHA = 0.5    # Adjusts smoothing. 1.0 = no smoothing, 0.1 = extremely smooth/slow response

# ============================================================
#    MAIN LOOP
# ============================================================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
time.sleep(0.5)

print("Robot Online. Tracking Red + ArUco...")

try:
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # --- 1. ARUCO DETECTION ---
        action = None
        if (time.time() - last_turn_time) > TURN_COOLDOWN:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if OLD_ARUCO:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
            else:
                corners, ids, _ = aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                tid = ids[0][0] # Focus on the first detected tag
                if tid in TAG_MAP:
                    action = TAG_MAP[tid]
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --- 2. EXECUTE ARUCO ACTION ---
        if action is not None:
            # Common sequence: STOP → FORWARD → ACTION
            print(f"ArUco Detected: {action}")
            
            # 1. Stop for 3 seconds
            stop_motor()
            time.sleep(3.0)
            
            # 2. Move forward for 1 second
            set_motor(BASE_SPEED, BASE_SPEED)
            time.sleep(0.7)
            
            # 3. Execute the action
            if action == "STRAIGHT":
                set_motor(BASE_SPEED, BASE_SPEED)
                time.sleep(1.0)
            elif action == "LEFT":
                turn_until_red("LEFT", cap)
            elif action == "RIGHT":
                turn_until_red("RIGHT", cap)
            
            last_turn_time = time.time()

        # --- 3. RED LINE FOLLOWING (Default Action) ---
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                                  cv2.inRange(hsv, lower_red2, upper_red2))
            
            # Look at the lower half only to stay focused on the track
            roi_mask = mask[FRAME_HEIGHT//2:, :]
            M = cv2.moments(roi_mask)
            
            if M["m00"] > 500:
                cX = int(M["m10"] / M["m00"])
                raw_error = cX - (FRAME_WIDTH // 2)
                
                # 1. EMA Filter: Smooth out camera sensor noise/jitter over time
                ema_error = (ERROR_ALPHA * raw_error) + ((1.0 - ERROR_ALPHA) * ema_error)
                
                # 2. PD Control for smooth steering
                derivative = ema_error - last_error
                last_error = ema_error
                
                steer = (STEER_GAIN * ema_error) + (STEER_D * derivative)
                
                # 3. Dynamic speed (keep forward momentum so it doesn't jerk to a halt)
                # Multiplier is now 0.8 instead of 1.2 to stop dramatic slowdowns,
                # and heavily clamps the lowest possible forward speed so it rolls through curves.
                forward_speed = max(25, BASE_SPEED - abs(steer) * 0.8)
                
                # Steer toward the red blob with pivoting logic
                set_motor(forward_speed + steer, forward_speed - steer)
                
                # Visual Indicator
                cv2.circle(frame, (cX, FRAME_HEIGHT - 30), 10, (0, 255, 0), -1)
            else:
                # No red? Continue forward instead of pivoting in place
                set_motor(BASE_SPEED, BASE_SPEED)
                cv2.putText(frame, "SEARCHING FOR RED (DRIVING FORWARD)", (20, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # Show the camera feed
        #cv2.imshow("Robot View", frame)
        #if cv2.waitKey(1) == ord('q'): break

except KeyboardInterrupt:
    print("\nUser Stopped Program.")
finally:
    print("Stopping motors and cleaning up...")
    stop_motor()
    time.sleep(0.2) # Give hardware time to receive the 0 PWM setting before exit
    try:
        left_f.close()
        left_b.close()
        right_f.close()
        right_b.close()
    except Exception as e:
        print(f"Error closing motor pins: {e}")
        
    if 'cap' in locals() and cap is not None:
        cap.release()
