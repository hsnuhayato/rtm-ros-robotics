# -*- coding: utf-8 -*-
from recognize import *

finishblock=[]

def plan(Tw_p,index): #Tw_p_list ---(eg) [["1",Tw_p-1],["3",Tw_p-3]]
#    piecelist=[]#    Tw_p = Tw_p_list[0][1] #TODO
#    index=inum#    index = Tw_p_list[0][0]
#    for l in Tw_p_list:
#        if l[0]==inum:
#            Tw_p=l[1]
#        else: piecelist.append(l)


    gmodel=self.gmodels[index]
#    Tworld_goal=piecelist[0][1]
    Tworld_goal=eye(4)
    Tworld_goal[0][3]=0
    Tworld_goal[1][3]=-0.07
    Tworld_goal[2][3]=0.1
    #    Tworld_goal[2][3]+=100
    success = False
    for validgrasp,validindex in gmodel.validGraspIterator():
        Tw_h2 = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
        Tw_h1 = dot(Tworld_goal, dot(inverse_matrix(Tw_p),Tw_h2)) # TODO
        with orrobot:
            gmodel.setPreshape(validgrasp)
            with gmodel.target:
                gmodel.target.SetTransform(Tworld_goal)
                sol = manip.FindIKSolution(Tw_h1,IkFilterOptions.CheckEnvCollisions)
                if sol is not None:
                    contacts, finalconfig, mindist, volume = gmodel.runGraspFromTrans(validgrasp)
                    orrobot.SetJointValues(finalconfig[0][manip.GetGripperIndices()], manip.GetGripperIndices())
                    gmodel.target.Enable(False)
                    sol = manip.FindIKSolution(Tw_h1,IkFilterOptions.CheckEnvCollisions)
                    if sol is not None:
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

def pick(LorR="L"):
    trajdata = taskmanip.CloseFingers(execute=False,outputtraj=True)[1]
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)
    grippervalues = traj.GetConfigurationSpecification().ExtractJointValues(traj.GetWaypoint(-1),orrobot,manip.GetGripperIndices(),0)

    with orrobot:

        orrobot.SetJointValues(grippervalues, manip.GetGripperIndices())
        if LorR == "L":
            angles = rr.get_joint_angles()
            alljointslist = orrobot.GetJoints() + orrobot.GetPassiveJoints()
            lhandlist = ["LHAND_JOINT0","LHAND_JOINT1","LHAND_JOINT2","LHAND_JOINT3"]
            for handname in lhandlist:
                angles[orderednames.index(handname)] = [(x.GetName(),x.GetValues()[0])  for x in alljointslist if x.GetName() == handname][0][1]
            rr.send_goal(angles,5.0,True)


def release(LorR="L"):
    if LorR == "L":
        #interfaces.CloseFingers()
        angle = 0.62
        angles = rr.get_joint_angles()
        angles[orderednames.index("LHAND_JOINT0")] = angle
        angles[orderednames.index("LHAND_JOINT1")] = -angle
        angles[orderednames.index("LHAND_JOINT2")] = -angle
        angles[orderednames.index("LHAND_JOINT3")] = angle
        rr.send_goal(angles,5.0,True)
        orrobot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])

pickuporder= [5,6,2,4,1,3,0]

#手の経路上の点のリストを入れると順に実行していく

def plan2(Tw_h1):
    trajdata = basemanip.MoveToHandPosition(matrices=[Tw_h1],execute=False,outputtraj=True)
    traj = RaveCreateTrajectory(env,'').deserialize(trajdata)
    return traj

def check(arg):
        finishblock=arg
        seelist = recognize()
        unvisibleblock=[0,1,2,3,4,5,6]
        if seelist != []:
            for see in seelist:
                if see[0] in unvisibleblock:
                    del unvisibleblock[unvisibleblock.index(see[0])]
                    print unvisibleblock
        for i in finishblock:
            if i in unvisibleblock:
                print i
                del unvisibleblock[unvisibleblock.index(i)]
                print unvisibleblock
        for i in unvisibleblock:
            tmp=Tworld_goal.copy()
            tmp[2][3]=2;
            self.gmodels[i].target.SetTransform(tmp)
#            env.Remove(self.gmodels[i].target)
        for i in finishblock:
            self.gmodels[i].target.SetTransform(Tworld_goal)


def compose(realrobot=False):
    for p in pickuporder:
        seelist = recognize()

        unvisibleblock=[0,1,2,3,4,5,6]
        if seelist == []:
            for see in seelist:
                if see[0] in unvisibleblock:
                    del unvisibleblock[unvisibleblock.index(see[0])]
        for i in finishblock:
            if i in unvisibleblock:
                del unvisibleblock[unvisibleblock.index(i)]

        for i in unvisibleblock:
            env.Remove(self.gmodels[i].target)

        for i in finishblock:
            self.gmodels[i].target.SetTransform(Tworld_goal)

        try:
            Tw_p = [i[1] for i in seelist if i[0] == p][0]
        except:
            print "No piece  can be grabbed!"
            #search()
        (traj,d1,Tw_h1,d2) = plan(Tw_p,p)
        execute(traj,realrobot=True,grasppiece=True)
        rr.send_goal(detectpose2,3.0,True)
        orrobot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])
        traj=plan2(Tw_h1)
        execute(traj,True,False)
        rr.send_goal(detectpose,3.0,True)
        orrobot.SetDOFValues(array(rr.get_joint_angles())[ordertoopenrave])



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
            rr.send_goal(q, 40.0*dt, True)
    # optional
    orrobot.GetController().SetPath(traj)
    orrobot.WaitForController(0)
    orrobot.GetController().Reset(0)
    if grasppiece:
        pick("L")
    else:
        release("L")
