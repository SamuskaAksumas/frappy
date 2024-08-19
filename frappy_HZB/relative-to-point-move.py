from urx import URRobot
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from coordinate_change import transform_point_to_global#,euler_to_rotation_matrix


HOST = "192.168.113.215"

robot = URRobot(HOST)#, urFirm=5.1)


print(robot.getl())

a=0.1
v=0.1
cur_pose = robot.getl()

rel_pose = [0, 0, 0.05, 0, 0, 0]
tpose_abs = [-0.1368, -0.2671, 0.2756, -0.0827, -2.8463, 0.8225]
tpose_start = [-0.1368, -0.2671, 0.3756, -0.0827, -2.8463, 0.8225]

time.sleep(2)

robot.movel(tpose_start, a, v)

robot.movel(tpose_abs, a, v)

#tpose = [v + tpose_abs[i] for i, v in enumerate(rel_pose)]
tpose = transform_point_to_global( rel_pose, tpose_abs)
robot.movel(tpose, a, v)#, relative=True)

# tpose = [v + tpose_abs[i] for i, v in enumerate([-1*x for x in rel_pose])]
# robot.movel(tpose, a, v)#, relative=True)

robot.movel(tpose_start,a,v)


robot.close()