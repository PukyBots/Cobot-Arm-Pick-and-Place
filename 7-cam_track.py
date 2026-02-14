import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import time

# --- MyCobot setup ---
mc = MyCobot(PI_PORT, PI_BAUD)
time.sleep(2)

# --- HSV color ranges ---
COLOR_RANGES = {
    "blue":   ([np.array([75, 150, 0]),   np.array([130, 255, 255])]),
    "green":  ([np.array([35, 100, 100]), np.array([85, 255, 255])]),
    "yellow": ([np.array([20, 100, 100]), np.array([30, 255, 255])]),
    "red1":   ([np.array([0, 120, 70]),   np.array([10, 255, 255])]),
    "red2":   ([np.array([170, 120, 70]), np.array([180, 255, 255])]),
}

# --- Open webcam ---
vs = cv2.VideoCapture(0)
vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

width  = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT))

frame_center_x = width // 2
frame_center_y = height // 2

# --- Scaling & smoothing ---
scale_x = 0.05  # mm per pixel
alpha = 0.4     # smoothing factor
tolerance_mm = 0.1  # dead zone in mm

# --- Joint limits ---
min_angle, max_angle = -65.0, 65.0

# --- Proportional gain ---
K = 8.0           # degrees per mm (tune for responsiveness)
J5_center = 0.0   # angle when object is exactly at frame center

# --- State ---
prev_cx = None

# --- Helper function ---
def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

mc.send_angles([90, 0, 0, 0, 0, 0], 50)
print(mc.is_paused())
time.sleep(2.5)

while True:
    ret, frame = vs.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    detected_color = None
    largest_area = 0
    best_box = None
    best_cnt = None

    # --- Detect colors except red ---
    for color, bounds in COLOR_RANGES.items():
        if color in ("red1", "red2"):
            continue
        lower, upper = bounds
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000 and area > largest_area:
                largest_area = area
                x, y, w, h = cv2.boundingRect(cnt)
                best_box = (x, y, w, h)
                best_cnt = cnt
                detected_color = color

    # --- Detect red separately ---
    lower1, upper1 = COLOR_RANGES["red1"]
    lower2, upper2 = COLOR_RANGES["red2"]
    mask_red = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 2000 and area > largest_area:
            largest_area = area
            x, y, w, h = cv2.boundingRect(cnt)
            best_box = (x, y, w, h)
            best_cnt = cnt
            detected_color = "red"

    # --- Process detected object ---
    if detected_color and best_box and best_cnt is not None:
        x, y, w, h = best_box
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, detected_color.upper(), (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])

            # --- Smooth object center ---
            if prev_cx is None:
                prev_cx = cx
            else:
                cx = int(alpha * cx + (1 - alpha) * prev_cx)
                prev_cx = cx

            # Draw for visualization
            cv2.circle(frame, (cx, frame_center_y), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"({cx},{frame_center_y})", (cx+10, frame_center_y+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 255), -1)
            cv2.putText(frame, "Center", (frame_center_x+10, frame_center_y+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


            # --- Simple proportional control (absolute target) ---
            error_mm = (cx - frame_center_x) * scale_x

            if abs(error_mm) > tolerance_mm:
                angles = mc.get_angles()
                if isinstance(angles, (list, tuple)) and len(angles) == 6:
                    target_J5 = clamp(J5_center + K * error_mm, min_angle, max_angle)
                    mc.send_angle(5, target_J5, 60)  # increase speed as needed

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

vs.release()
cv2.destroyAllWindows()
