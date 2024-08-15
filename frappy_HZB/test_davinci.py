from gripper_davinci import Gripper
import asyncio
from urx import URRobot

HOST = "169.254.60.223"
gripper = Gripper(HOST, 63352)


# Roboterverbindung
# done=False
# i=0
# while not done:
#     if input() == 'q':
#         break
#     try:
robot = URRobot(HOST, urFirm=5.9)
    #     done = True
    # except:
    #     i += 1
    #     print('Failed connections:',i)

a=0.1
v=0.1

async def log_info(gripper):
    print(f"Pos: {str(await gripper.get_current_position()): >3}  "
          f"Open: {await gripper.is_open(): <2}  "
          f"Closed: {await gripper.is_closed(): <2}  ")

async def run():
    await gripper.connect()
    await gripper.activate(auto_calibrate=True)
    #await print('Min Pos:', gripper.get_min_position)
    #await print('Is open:', gripper.is_open)
    #await print('Is active:', gripper.is_active)
    await gripper.move_and_wait_for_pos(1, 100, 100)
    await gripper.move_and_wait_for_pos(255, 100, 100)
    print(await gripper.get_current_position())
    print('detected:', await gripper._get_var('OBJ'))
    robot.movel((0, 0, 0.05, 0, 0, 0), a, v, relative=True)
    robot.movel((0, 0, -0.05, 0, 0, 0), a, v, relative=True)
    await gripper.move_and_wait_for_pos(1,100,100)
    await gripper.disconnect()
    robot.close()
    

asyncio.run(run())

