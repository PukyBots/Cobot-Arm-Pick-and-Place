import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# ------------- USER PARAMETERS -------------
ROBOT_PORT = '/dev/ttyAMA0'
ROBOT_BAUD = 1000000
TABLE_Z = 0.0
PRE_PICK_Z = 50.0  # Safe hover height
MIN_AREA = 2000    # Minimum contour area to consider

# ------------- HSV COLOR RANGES -------------
COLOR_RANGES = {
    "blue":   ([np.array([75, 150, 0]),   np.array([130, 255, 255])]),
    "green":  ([np.array([35, 100, 100]), np.array([85, 255, 255])]),
    "yellow": ([np.array([20, 100, 100]), np.array([30, 255, 255])]),
    "red1":   ([np.array([0, 120, 70]),   np.array([10, 255, 255])]),
    "red2":   ([np.array([170, 120, 70]), np.array([180, 255, 255])]),
}

# ------------- HOMOGRAPHY CALIBRATION -------------
pixels = np.array([
    [400, 23],
    [400, 260],
    [215, 23],
    [215, 260],
], dtype=float)

world_coords = np.array([
    [62.0, -120.0],
    [62.0, 120.0],
    [240.0, -120.0],
    [240.0, 120.0],
], dtype=float)

H, _ = cv2.findHomography(pixels, world_coords)

# ------------- INIT ROBOT -------------
mc = MyCobot(ROBOT_PORT, ROBOT_BAUD)

# ------------- CAMERA INIT -------------
vs = cv2.VideoCapture(0)
vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

width  = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_center_x = width // 2
frame_center_y = height // 2

# Smoothing
prev_cx, prev_cy = None, None
alpha = 0.4

# Trigger lock to avoid repeated pickups
last_pick_time = 0
cooldown_time = 5  # seconds


# ------------- CONVERSION FUNCTION -------------
def pixel_to_table(uv, H, table_z=TABLE_Z):
    uv_h = np.array([uv[0], uv[1], 1.0])
    XY_h = H @ uv_h
    XY_h /= XY_h[2]
    X, Y = XY_h[0], XY_h[1]
    return np.array([X, Y, table_z])


def move_to_position(target_xyz, pre_z=PRE_PICK_Z):
    mc.send_coords([target_xyz[0], target_xyz[1], pre_z, -170.0, -1.5, 122.0], 50, 0)
    time.sleep(1)
    mc.send_coords([target_xyz[0], target_xyz[1], target_xyz[2], -170.0, -1.5, 122.0], 30, 0)
    time.sleep(1)
    mc.send_coords([target_xyz[0], target_xyz[1], pre_z, -170.0, -1.5, 122.0], 50, 0)
    time.sleep(1)


# ------------- MAIN LOOP -------------
while True:
    ret, frame = vs.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_color = None
    largest_area = 0
    best_cnt = None
    best_box = None

    # Handle all colors except red
    for color, bounds in COLOR_RANGES.items():
        if color in ("red1", "red2"):
            continue
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

    # Special red case
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

    # If object found, act
    if detected_color and best_box and best_cnt is not None:
        x, y, w, h = best_box
        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            if prev_cx is None:
                prev_cx, prev_cy = cx, cy
            else:
                cx = int(alpha * cx + (1 - alpha) * prev_cx)
                cy = int(alpha * cy + (1 - alpha) * prev_cy)
                prev_cx, prev_cy = cx, cy

            # Draw visual feedback
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"{detected_color.upper()} ({cx}, {cy})", (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Trigger robot move (if cooldown passed)
            current_time = time.time()
            if current_time - last_pick_time > cooldown_time:
                target_xyz = pixel_to_table((cx, cy), H)
                print(f"[{detected_color.upper()}] Moving to: {target_xyz}")
                move_to_position(target_xyz)
                last_pick_time = current_time

    cv2.imshow("Detection", frame)
    key = cv2.waitKey(1)
    if key == 27:  # ESC key to quit
        break

vs.release()
cv2.destroyAllWindows()
