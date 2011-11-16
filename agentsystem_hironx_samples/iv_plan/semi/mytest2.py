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

manip=robot.SetActiveManipulator("rightarm_torso")
ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,freeindices=manip.GetArmIndices()[:-6])
if not ikmodel.load():
    ikmodel.autogenerate()
    
def recognize():
    pose = rr.recognize('rhand')[0][1]
    f = ctsvc.ref.Query('rhandcam', pose2mat(pose), robotframe, rr.get_joint_angles())
    f = reshape(f, (4,4))
    f[0:3,3] /= 1000 # hand goal
    return f

def pick(Tgoal):
    # openravepy
    basemanip = interfaces.BaseManipulation(robot)
    trajdata = basemanip.MoveToHandPosition(matrices=[Tgoal],execute=False,outputtraj=True)
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)
    return traj

def execute(traj):
    robot.GetController().SetPath(traj)

    spec = traj.GetConfigurationSpecification()
    data = traj.GetWaypoint(0)
    rtcvalues = spec.ExtractJointValues(data,robot,ordertoopenrave,0)
    spec.ExtractDeltaTime(data)

    waypointoffset=spec.GetGroupFromName("iswaypoint").offset
    if data[waypointoffset]:
        print "is waypoint"
    else:
        print "can be ignored"
    print traj.GetNumWaypoints()
    print traj.GetWaypoint(0)
