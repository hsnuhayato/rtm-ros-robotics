# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from hironx_if import *

from openravepy import *
from numpy import *
import cubeassembly

env=Environment()
env.SetViewer('qtcoin')
env.Load('hironxtable.env.xml')
orrobot=env.GetRobots()[0]
T = orrobot.GetTransform()
T[2,3] = 0.09
orrobot.SetTransform(T)

rr.connect()

detectpose = [0.0, 0.0, 1.1,
              0.70, -0.38, -2.23, 0.52, 1.16, -0.23,
              -0.42, -0.42, -2.31, -0.69, 1.20, -0.22,
              0.64, -0.63, -0.62, 0.63,
              0.62, -0.63, -0.64, 0.64]
detectpose2 = [0.0, 0.0, 1.1,
              0.70, -0.38, -2.23, 0.52, 1.16, -0.23,
              -0.42, -0.42, -2.31, -0.69, 1.20, -0.22,
              0.64, -0.63, -0.62, 0.63,
              -6*pi/180, 6*pi/180, 6*pi/180 ,-6*pi/180]

orderednames = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'RHAND_JOINT0', 'RHAND_JOINT1', 'RHAND_JOINT2', 'RHAND_JOINT3', 'LHAND_JOINT0', 'LHAND_JOINT1', 'LHAND_JOINT2', 'LHAND_JOINT3']
ordertoopenrave = [orderednames.index(j.GetName()) for j in orrobot.GetJoints()]

# 受動関節がー１の値をとるのでorrobot.GetPassiveJoints()で処理してください
# orrobot.GetPassiveJoints()[0].GetValues()
ordertortc = array([orrobot.GetJoint(name).GetDOFIndex() for name in orderednames],int32)

orrobot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])

manip=orrobot.SetActiveManipulator("leftarm_torso")
ikmodel=databases.inversekinematics.InverseKinematicsModel(orrobot,freeindices=manip.GetArmIndices()[:-6])
# if not ikmodel.load():
#     ikmodel.autogenerate()

self = cubeassembly.CubeAssembly(orrobot)
self.CreateBlocks()

Tworld_goal = eye(4)
Tworld_goal[0:3,3] = [0.4,0,0] # 編集が必要です

import recognize
import plan


