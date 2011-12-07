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

def recognize(camera='lhand'):
#マーカとブロックの座標の対応配列 [ブロック番号,原点のずれ,回転のずれ（オイラー角）]
    Tp_m_list=[ [4,[0.045,-0.015,0.09],[pi,0,0]],
                [1,[-0.015,0.015,0.06],[pi,0,0]],
                [5,[0.015,0.015,0.09],[0,0,0]] ]
    id_index=[7,2,23]#Tp_m_listの各行のマーカのid

    zxz=[[0,0,1],[0,1,0],[0,0,1]]#オイラー角の回転軸ベクトル

    poselist=rr.recognize(camera)

    f=[]
    Tw_p_list=[]#Tw_pのリスト
    for i in range(0,len(poselist)):#i:何番目に認識したブロックか
        f.append( ctsvc.ref.Query(camera+'cam', pose2mat(poselist[i][1]), robotframe, rr.get_joint_angles()))
        f[i] = reshape(f[i], (4,4))

        # vpython env => openrave env
        relvec = array([150,0,0]) + array([-450,0,-710])
        f[i][0:3,3] += relvec
        f[i][0:3,3] /= 1000
                
        # marker => piece
        Tp_m = eye(4)
        for j in range(0,3):
            Tp_m[0:3,0:3] *= rotationMatrixFromAxisAngle(zxz[j], Tp_m_list[ id_index.index(poselist[i][0]) ] [2] [j]  )
        Tp_m[0:3,3] = Tp_m_list[ id_index.index(poselist[i][0]) ][1]

        # f * Tp_m^-1
        Tw_p = dot(f[i],inverse_matrix(Tp_m))
        # Tw_p_list[i][1]=Tw_p

        # Tw_p_list[i][0]=Tp_m_list[ id_index.index(poselist[i][0]) ][0]

        Tw_p_list.append([Tp_m_list[ id_index.index(poselist[i][0]) ][0],Tw_p])
    # aqua piece
        self.gmodels[Tw_p_list[i][0]].target.SetTransform(Tw_p_list[i][1])
    # self.gmodels[4].target.SetTransform(Tw_p)
    return Tw_p_list

deletedlist = []

def plan1(Tw_p_list): #Tw_p_list ---(eg) [["1",Tw_p-1],["3",Tw_p-3]]
    Tw_p = Tw_p_list[0][1] #TODO
    index = Tw_p_list[0][0]
    for l in Tw_p_list:
        if l[0]==4:
            Tw_p=l[1]
        else: deletedlist.append(l)

    gmodel=self.gmodels[index]
    success = False
    for validgrasp,validindex in gmodel.validGraspIterator():
        # 目的地への検証：目的地計算、逆運動学
        Tw_h2 = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
        Tw_h1 = eye(4) # TODO
        with orrobot:
            gmodel.setPreshape(validgrasp)
            with gmodel.target:
                gmodel.target.SetTransform(Tworld_goal)
                sol = manip.FindIKSolution(Tw_h1,IkFilterOption.CheckEnvCollision)
                if sol is None:
                    success = True
                    break
                
    assert(success)
    gmodel.showgrasp(validgrasp) # show the grasp

    trajdata = basemanip.MoveToHandPosition(matrices=[Tw_h2],execute=False,outputtraj=True)
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)
    return traj,Tw_h2,Tw_h1

    #self.gmodelsから計算されます
    #4番のpieceを捕むためのハンドのワールド座標を生成
    #4番以外のpieceが邪魔にならないかどうか調べる
    
#     #まず目標座標
#     Tp_h = eye(4)
#     Tp_h[0:3,3] = [0.045,-0.015,0.12+0.059]
#     f = dot(Tw_p,Tp_h)
#     h = dot(dot(f[0:3,0:3], rotationMatrixFromAxisAngle([1,0,0],pi)), rotationMatrixFromAxisAngle([0,0,1],pi/2))
#     f[0:3,0:3] = h
#     Tw_h = f
# 
#     basemanip = interfaces.BaseManipulation(orrobot)
#     # trajdataはXML式です
#     # http://openrave.org/en/main/architecture/trajectory.html?highlight=trajectory%20xml
#     trajdata = basemanip.MoveToHandPosition(matrices=[Tw_h],execute=False,outputtraj=True)
#     traj = RaveCreateTrajectory(env,'').deserialize(trajdata)

    return Tw_h, traj

def plan2(Tw_p_list):
    #4番から他のpieceへの行列を求める
    Tw_p = deletedlist[0][1]
    Tp_h = eye(4)
    Tp_h[0:3,3] = [0.045,-0.015,0.12+0.059]
    f = dot(Tw_p,Tp_h)
    h = dot(dot(f[0:3,0:3], rotationMatrixFromAxisAngle([1,0,0],pi)), rotationMatrixFromAxisAngle([0,0,1],pi/2))
    f[0:3,0:3] = h
    Tw_h = f
    #Tw_h[2][3] += 30  # z方向に少し足しこむ(mm)
    basemanip = interfaces.BaseManipulation(orrobot)
    # trajdataはXML式です
    # http://openrave.org/en/main/architecture/trajectory.html?highlight=trajectory%20xml
    trajdata = basemanip.MoveToHandPosition(matrices=[Tw_h],execute=False,outputtraj=True)
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)

    return Tw_h, traj

def pick(LorR="L"):
    if LorR == "L":
        #interfaces.CloseFingers()
        angles = rr.get_joint_angles()
        angles[orderednames.index("LHAND_JOINT0")] = -6*pi/180
        angles[orderednames.index("LHAND_JOINT1")] = 6*pi/180
        angles[orderednames.index("LHAND_JOINT2")] = 6*pi/180 
        angles[orderednames.index("LHAND_JOINT3")] = -6*pi/180
        rr.send_goal(angles,5.0,True)
        

#手の経路上の点のリストを入れると順に実行していく
def compose(Tw_hboollist,realrobot=False):
    trajs=[]
    for Tw_h in Tw_hboollist:
        trajdata = basemanip.MoveToHandPosition(matrices=[Tw_h[0]],execute=False,outputtraj=True)
    # trajdataはXML式です
    # http://openrave.org/en/main/architecture/trajectory.html?highlight=trajectory%20xml
        traj = RaveCreateTrajectory(env,'').deserialize(trajdata)
        execute(traj,realrobot,Tw_h[1])


def execute(traj, realrobot=False,grasppiece=False):
    spec = traj.GetConfigurationSpecification()
    n = traj.GetNumWaypoints()

    for i in range(1,n):
        data = traj.GetWaypoint(i)
        rtcvalues = spec.ExtractJointValues(data,orrobot,ordertoopenrave,0)
        dt = spec.ExtractDeltaTime(data)
        print dt
        print data
        if realrobot:
            q = rr.get_joint_angles()
            q[0] = data[0]
            q[9:15] = data[1:7]
            rr.send_goal(q, 20.0*dt, True)
    # optional
    orrobot.GetController().SetPath(traj)
    orrobot.WaitForController(0)
    orrobot.GetController().Reset(0)
    if grasppiece:
        pick("L")

