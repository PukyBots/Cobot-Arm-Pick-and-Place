import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from pymycobot.mycobot import MyCobot

# ----------------- USER PARAMETERS -----------------
ROBOT_PORT = '/dev/ttyAMA0'
ROBOT_BAUD = 1000000
TABLE_Z = 0.0
PRE_PICK_Z = 200
PRE_DROP_Z = 200
MIN_AREA = 2000
robot_busy = False

# Belt & encoder setup
BELT_CIRCUMFERENCE = 400  # mm
PULSES_PER_ROTATION = 1000
displacement_per_pulse = BELT_CIRCUMFERENCE / PULSES_PER_ROTATION
ENCODER_PIN_A = 17
encoder_count = 0

# ----------------- BOXES -----------------
BOX_POSITIONS = {
    "red": [62.0, -175.0, 100.0],
    "green": [-22.0, -154.0, 100.0],
    "blue": [-107.0, -138.0, 100.0]
}

# ----------------- COLOR RANGES -----------------
COLOR_RANGES = {
    "blue": ([np.array([75, 150, 0]), np.array([130, 255, 255])]),
    "green": ([np.array([35, 100, 100]), np.array([85, 255, 255])]),
    "yellow": ([np.array([20, 100, 100]), np.array([30, 255, 255])]),
    "red1": ([np.array([0, 120, 70]), np.array([10, 255, 255])]),
    "red2": ([np.array([170, 120, 70]), np.array([180, 255, 255])]),
}

# ----------------- HOMOGRAPHY -----------------
pixels = np.array([[400,23],[400,260],[215,23],[215,260]], dtype=float)
world_coords = np.array([[62,-120],[62,120],[240,-120],[240,120]], dtype=float)
H, _ = cv2.findHomography(pixels, world_coords)

# ----------------- ENCODER CALLBACK -----------------
def encoder_callback(channel):
    global encoder_count
    encoder_count += 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODER_PIN_A, GPIO.RISING, callback=encoder_callback)

# ----------------- INIT ROBOT -----------------
mc = MyCobot(ROBOT_PORT, ROBOT_BAUD)

# ----------------- CAMERA -----------------
vs = cv2.VideoCapture(0)
vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ----------------- HELPER FUNCTIONS -----------------
def pixel_to_table(uv, H, table_z=TABLE_Z):
    uv_h = np.array([uv[0], uv[1], 1.0])
    XY_h = H @ uv_h
    XY_h /= XY_h[2]
    X, Y = XY_h[0], XY_h[1]
    return np.array([X, Y, table_z])

def pick_object(target_xyz):
    SAFE_OFFSET = 50
    approach_z = target_xyz[2] + SAFE_OFFSET
    mc.send_coords([target_xyz[0], target_xyz[1], PRE_PICK_Z, 0, 180, 0], 50, 0)
    time.sleep(0.2)
    mc.send_coords([target_xyz[0], target_xyz[1], approach_z, 0, 180, 0], 30, 0)
    time.sleep(0.2)
    # gripper_close()
    time.sleep(0.2)
    mc.send_coords([target_xyz[0], target_xyz[1], PRE_PICK_Z, 0, 180, 0], 50, 0)
    time.sleep(0.2)

def place_object(box_xyz):
    SAFE_OFFSET = 50
    approach_z = box_xyz[2] + SAFE_OFFSET
    mc.send_coords([box_xyz[0], box_xyz[1], PRE_DROP_Z, 0, 180, 0], 50, 0)
    time.sleep(0.5)
    mc.send_coords([box_xyz[0], box_xyz[1], approach_z, 0, 180, 0], 30, 0)
    time.sleep(0.5)
    # gripper_open()
    time.sleep(0.5)
    mc.send_coords([box_xyz[0], box_xyz[1], PRE_DROP_Z, 0, 180, 0], 50, 0)
    time.sleep(0.5)

def return_home():
    mc.send_coords([55.7, -69.6, 416.7, -82.41, -15.87, -98.69], 50, 0)
    time.sleep(0.5)

# ----------------- DETECTION FUNCTION -----------------
def detect_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_color = None
    largest_area = 0
    best_cnt = None
    best_box = None

    for color, bounds in COLOR_RANGES.items():
        if color in ("red1", "red2"): continue
        lower, upper = bounds
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA and area > largest_area:
                largest_area = area
                best_cnt = cnt
                x, y, w, h = cv2.boundingRect(cnt)
                best_box = (x, y, w, h)
                detected_color = color

    # Red case
    lower1, upper1 = COLOR_RANGES["red1"]
    lower2, upper2 = COLOR_RANGES["red2"]
    mask_red = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > MIN_AREA and area > largest_area:
            largest_area = area
            best_cnt = cnt
            x, y, w, h = cv2.boundingRect(cnt)
            best_box = (x, y, w, h)
            detected_color = "red"

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            return cx, cy, detected_color
    return None

# ----------------- MAIN LOOP -----------------
prev_cx, prev_cy = None, None
while True:
    if not robot_busy:
        # Flush old frames
        for _ in range(3):
            vs.grab()
        ret, frame = vs.read()
        if not ret:
            break

        detected = detect_object(frame)
        if detected:
            cx, cy, color = detected
            # Smooth centroid
            if prev_cx is None:
                prev_cx, prev_cy = cx, cy
            else:
                alpha = 0.4
                cx = int(alpha*cx + (1-alpha)*prev_cx)
                cy = int(alpha*cy + (1-alpha)*prev_cy)
                prev_cx, prev_cy = cx, cy

            # Initial detection
            X0, Y0, Z0 = pixel_to_table((cx, cy), H)
            encoder_start = encoder_count
            robot_busy = True

            # --- Predict moving position ---
            while True:
                displacement = (encoder_count - encoder_start) * displacement_per_pulse
                X_pred = X0 + displacement
                target_xyz = [X_pred, Y0, Z0]

                # Pick if within reachable area
                if X_pred >= 60:  # adjust threshold for your robot
                    pick_object(target_xyz)
                    box_xyz = BOX_POSITIONS[color]
                    place_object(box_xyz)
                    return_home()
                    robot_busy = False
                    break

    # Display camera for debugging
    ret, disp_frame = vs.read()
    if ret:
        cv2.imshow("Detection", disp_frame)
    if cv2.waitKey(1) & 0xFF == 27:
        mc.release_all_servos()
        break

vs.release()
GPIO.cleanup()
cv2.destroyAllWindows()
