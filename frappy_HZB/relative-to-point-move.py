from urx import URRobot

HOST = "169.254.60.223"

robot = URRobot(HOST, urFirm=5.1)

a=0.1
v=0.1
cur_pose = robot.getl()

rel_pose = [0, 0, 0.05, 0, 0, 0]
tpose_abs = [0.6685, 0.1448, 0.20494, -2.1120, 0.1209, -2.1727]
tpose = [v + tpose_abs[i] for i, v in enumerate(rel_pose)]
robot.movel(tpose, a, v)#, relative=True)
tpose = [v + tpose_abs[i] for i, v in enumerate([-1*x for x in rel_pose])]
robot.movel(tpose, a, v)#, relative=True)

robot.movel(cur_pose,a,v)


robot.close()