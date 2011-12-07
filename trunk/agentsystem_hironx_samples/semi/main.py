# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from hironx_if import *

from openravepy import *
from numpy import *
import cubeassembly

from init import *

T = orrobot.GetTransform()
T[2,3] = 0.09
orrobot.SetTransform(T)

rr.connect()


# 受動関節がー１の値をとるのでorrobot.GetPassiveJoints()で処理してください
# orrobot.GetPassiveJoints()[0].GetValues()
ordertortc = array([orrobot.GetJoint(name).GetDOFIndex() for name in orderednames],int32)

orrobot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])


ikmodel=databases.inversekinematics.InverseKinematicsModel(orrobot,freeindices=manip.GetArmIndices()[:-6])
# if not ikmodel.load():
#     ikmodel.autogenerate()

from recognize import * 
from plan import *

recognize.rr=rr
