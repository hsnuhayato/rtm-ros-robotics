# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from hironx_if import *

from cubeassembly import *

env=Environment()
env.SetViewer('qtcoin')
env.Load('data/hironxtable.env.xml')
robot=env.GetRobots()[0]
rr.connect()

orderednames = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1', 'RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'RHAND_JOINT0', 'RHAND_JOINT1', 'RHAND_JOINT2', 'RHAND_JOINT3', 'LHAND_JOINT0', 'LHAND_JOINT1', 'LHAND_JOINT2', 'LHAND_JOINT3']
ordertortc = array([robot.GetJoint(name).GetDOFIndex() for name in orderednames],int32)
ordertoopenrave = [orderednames.index(j.GetName()) for j in robot.GetJoints()]
robot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])

manip=robot.SetActiveManipulator("leftarm_torso")
ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,freeindices=manip.GetArmIndices()[:-6])
if not ikmodel.load():
    ikmodel.autogenerate()

self = CubeAssembly(robot)
self.CreateBlocks()


def recognize(camera='lhand'):
    pose = rr.recognize(camera)[0][1]
    f = ctsvc.ref.Query(camera+'cam', pose2mat(pose), robotframe, rr.get_joint_angles())
    f = reshape(f, (4,4))

    relvec = array([150,0,0]) + array([-450,0,-800])
    f[0:3,3] += relvec
    f[0:3,3] /= 1000

    self.gmodels[4].target.SetTransform(f)
    return f

def plan(f):
    g = eye(4)
    g[0:3,3] = [0.03,-0.03,0.12]
    f = dot(f,g)
    h = dot(dot(f[0:3,0:3], rotationMatrixFromAxisAngle([1,0,0],pi)), rotationMatrixFromAxisAngle([0,0,1],-pi/2))
    f[0:3,0:3] = h
    Tgoal = f

    basemanip = interfaces.BaseManipulation(robot)
    trajdata = basemanip.MoveToHandPosition(matrices=[Tgoal],execute=False,outputtraj=True)
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)

    return f, traj

def execute(traj, realrobot=False):
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)
    robot.GetController().Reset(0)

    spec = traj.GetConfigurationSpecification()
    n = traj.GetNumWaypoints()
    for i in range(n):
        data = traj.GetWaypoint(i)
        rtcvalues = spec.ExtractJointValues(data,robot,ordertoopenrave,0)
        dt = spec.ExtractDeltaTime(data)
        print dt
        print rtcvalues
