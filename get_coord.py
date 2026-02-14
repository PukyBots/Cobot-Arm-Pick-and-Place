from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  
import time

mc = MyCobot(PI_PORT, PI_BAUD)


print(mc.get_angles())
print(mc.get_coords())
# print(mc.get_angles_coords())
time.sleep(0.5)
mc.send_coord(1,340,20)
time.sleep(2.5)
mc.send_angle(1,90,20)
time.sleep(2.5)
print(mc.get_angles())
print(mc.get_coords())
mc.send_coords([300,200,140,50,0,0],40,1)
time.sleep(2.5)
print(mc.get_coords())