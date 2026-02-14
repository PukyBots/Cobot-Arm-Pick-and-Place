from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  
import time

mc = MyCobot(PI_PORT, PI_BAUD)


# mc.send_coor 


coords=mc.get_coords()
print(f"{coords}")

time.sleep(2)

mc.release_all_servos()