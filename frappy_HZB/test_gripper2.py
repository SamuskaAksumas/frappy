#from urx.gripper import Gripper
import urx
#from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from urx.robotiqGripper import RobotiqGripper
import numpy as np
import time
import keyboard
import asyncio

TCP_Robot = "192.168.2.2"
TCP_computer = "192.168.8.1"      # IP-Adresse meines Rechners (ich muss manuell einen Link einrichten
                                    # und schreiben Sie eine IP-Adresse in dasselbe Subnetz
                                    # als Roboter 192.168.1.XX)
TCP_PORT_GRIPPER = 40001            # nicht ändern
TCP_PORT = 30002                    # nicht ändern

# Roboterverbindung
done=False
i=0
while not done:
    if input() == 'q':

        break
    try:
        robot = urx.URRobot(TCP_Robot, urFirm=5.1)
        done = True
    except:
        i += 1
        print('Failed connections:',i)
    
robot.set_tcp((0, 0, 0.15, 0, 0, 0))     # Position des Werkzeugmittelpunktes relativ zur Spitze des Roboters (x,y,z,Rx,Ry,Rz)
                                        # (in mm und radians)
robot.set_payload(1.12, (3, -2, 50))       # Gewicht des Werkzeugs und Lage des Schwerpunkts (in kg und mm)

# Geiferverbindung
gripper = RobotiqGripper(TCP_Robot)

# Bewegungseinstellungen
acc = 0.4                           # maximale Gelenkbeschleunigung
vel = 0.4                           # maximale Gelenkgeschwindigkeit
deg2rad = np.pi/180

print(robot.getl())                                             # Gibt die (x, y, z) des TCP-Werts zurück.
robot.movel([0.05,0,0,0,0,0],acc, vel, relative=True)
robot.movel([-0.05,0,0,0,0,0],acc, vel, relative=True)

print(robot.getl())




# Greiferbefehle
async def log_info(gripper):
    print(f"Pos: {str(await gripper.get_current_position()): >3}  "
          f"Open: {await gripper.is_open(): <2}  "
          f"Closed: {await gripper.is_closed(): <2}  ")

async def run():
    #gripper = Gripper(TCP_Robot,TCP_PORT)  # actual ip of the ur arm

    await gripper.connect()
    await gripper.activate()  # calibrates the gripper

    await gripper.move_and_wait_for_pos(255, 255, 255)
    await log_info(gripper)
    await gripper.move_and_wait_for_pos(0, 255, 255)
    await log_info(gripper)
    #await gripper.disconnect()

asyncio.run(run())

robot.close()                                                   # Schließen der Verbindung