from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  
import time

mc = MyCobot(PI_PORT, PI_BAUD)

# Release all servos at the start
mc.release_all_servos()
print("All servos released.")
time.sleep(1)

stored_positions = []
iterations = 10
wait_time_between_positions = 2  # less time for capturing

print("Starting to collect positions...")

for i in range(iterations):
    coords = mc.get_coords()
    print(f"Iteration {i+1}: Current coordinates: {coords}")
    
    stored_positions.append(coords)
    
    time.sleep(wait_time_between_positions)

print("Finished collecting positions.")
print(f"Stored positions: {stored_positions}")

# Execute all stored coordinates fast without delay
print("Starting execution of stored positions...")

start_time = time.time()

for idx, pos in enumerate(stored_positions):
    print(f"Moving to position {idx+1}: {pos}")
    mc.send_coords(pos, 50, 0)  # speed 50, mode 0
    # no delay here for fast execution

end_time = time.time()
duration = end_time - start_time

print(f"Completed execution of all positions in {duration:.2f} seconds.")
