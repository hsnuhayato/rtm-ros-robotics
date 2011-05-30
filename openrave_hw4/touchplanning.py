#!/usr/bin/env python
# -*- coding: utf-8 -*-
from openravepy import *
from numpy import *
from optparse import OptionParser

def showgrasp(manip,Tgrasp):
    """
    :param manip: が現在の使われるマニピュレーター
    :param Tgrasp: マニピュレーターの到達位置姿勢
    """
    robot = manip.GetRobot()
    with robot:
        Tdelta = dot(Tgrasp,linalg.inv(manip.GetEndEffectorTransform()))
        for link in manip.GetChildLinks():
            link.SetTransform(dot(Tdelta,link.GetTransform()))
        robot.GetEnv().UpdatePublishedBodies()
        raw_input('press any key')

if __name__ == "__main__":
    parser = OptionParser(description='touch planning script')
    parser.add_option('--scene', action="store",type='string',dest='scene',default='test.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manip', action="store",type='string',dest='manip',default='rightarm2',
                      help='Manipulator to use (default=%default)')
    (options, args) = parser.parse_args()

    env=Environment()
    env.SetViewer('qtcoin')
    env.Load(options.scene)
    
    robot=env.GetRobots()[0]
    manip=robot.SetActiveManipulator(options.manip)
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D,freeindices=[0])
    if not ikmodel.load():
        ikmodel.autogenerate()
    ikmodel.setrobot(freeinc=[0.04])
    basemanip = interfaces.BaseManipulation(robot)

    # 宿題：正しい目的地を計算する必要があります。
    # これはtest.env.xmlでしか動かないです。
    Thand = eye(4); Thand[0:3,3] = [0.18,-0.1,0.4]
    Thand2 = array([[0,1,0,0.2],[0,0,1,-0.04],[1,0,0,0.4],[0,0,0,1]])
    matrices = []
    with env:
        for angle in arange(0,2*pi,0.2):
            T = dot(Thand2,matrixFromAxisAngle([0,0,angle]))
            if not manip.CheckEndEffectorCollision(T):
                matrices.append(T)

    basemanip.MoveToHandPosition(matrices=matrices)
    robot.WaitForController(0)
    basemanip.MoveManipulator(zeros(7))
    robot.WaitForController(0)

    print 'showing grasps'
    for T in matrices:
        showgrasp(manip,T)
