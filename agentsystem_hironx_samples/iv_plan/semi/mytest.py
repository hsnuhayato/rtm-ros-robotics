# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from hironx_if import *

# openravepy
from openravepy import *
env=Environment()
env.SetViewer('qtcoin')
env.Load('robots/kawada-hironx.zae')
robot=env.GetRobots()[0]
rr.connect()

# write your own code

orderednames = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'RHAND_JOINT0', 'RHAND_JOINT1', 'RHAND_JOINT2', 'RHAND_JOINT3', 'LHAND_JOINT0', 'LHAND_JOINT1', 'LHAND_JOINT2', 'LHAND_JOINT3']
ordertortc = array([robot.GetJoint(name).GetDOFIndex() for name in orderednames],int32)
ordertoopenrave = [orderednames.index(j.GetName()) for j in robot.GetJoints()]
robot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])
