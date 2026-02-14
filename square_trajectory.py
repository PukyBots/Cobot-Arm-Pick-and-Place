import time
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

# ---------------- ROBOT SETUP ----------------
mc = MyCobot(PI_PORT, PI_BAUD)
time.sleep(2)
mc.power_on()
time.sleep(2)


SPEED = 10   # lower speed = smoother, cinematic


# ---------------- HOME POSITION ----------------
HOME = [0, 0, 0, 0, 0, 0]


# ---------------- BIG SQUARE JOINT POSES ----------------
# Big motion using J1, J2, J3 mainly


# Front – Left
P1 = [ -25, -45,  55,   5,  10,  0 ]


# Front – Right
P2 = [  25, -45,  55,   5,  10,  0 ]


# Back – Right
P3 = [  25, -20,  25,  10,  15,  0 ]


# Back – Left
P4 = [ -25, -20,  25,  10,  15,  0 ]


# ---------------- SMOOTH INTERPOLATION ----------------
def smooth_move(start, end, steps=50, delay=0.04):
    for i in range(steps + 1):
        r = i / steps
        angles = [
            start[j] + r * (end[j] - start[j])
            for j in range(6)
        ]
        mc.send_angles(angles, SPEED)
        time.sleep(delay)


# ---------------- EXECUTION ----------------
print("Moving to HOME")
smooth_move(HOME, P1, steps=60)


print("Drawing BIG square (cinematic)")
smooth_move(P1, P2)   # Horizontal front
smooth_move(P2, P3)   # Backward right
smooth_move(P3, P4)   # Horizontal back
smooth_move(P4, P1)   # Forward left


print("Returning to HOME")
smooth_move(P1, HOME, steps=60)


print("Big square completed")