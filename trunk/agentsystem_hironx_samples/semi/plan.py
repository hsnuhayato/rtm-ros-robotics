# -*- coding: utf-8 -*-
def plan1(Tw_p_list): #Tw_p_list ---(eg) [["1",Tw_p-1],["3",Tw_p-3]]
    piecelist=[]#    Tw_p = Tw_p_list[0][1] #TODO
    index=4#    index = Tw_p_list[0][0]
    for l in Tw_p_list:
        if l[0]==4:
            Tw_p=l[1]
        else: piecelist.append(l)
    gmodel=self.gmodels[index]
#    Tworld_goal=piecelist[0][1]
    Tworld_goal=eye(4)
    Tworld_goal[0:3][3]=[0.1,-0.07,0]#    Tworld_goal[2][3]+=100
    success = False
    for validgrasp,validindex in gmodel.validGraspIterator():
        # 目的地への検証：目的地計算、逆運動学
        Tw_h2 = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
        Tw_h1 = dot(Tworld_goal, dot(inverse_matrix(Tw_p),Tw_h2)) # TODO
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
    return traj,Tw_h2,Tw_h1,piece_list

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

#    return Tw_h, traj

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

