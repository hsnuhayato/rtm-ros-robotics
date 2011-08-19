##
## RRT-connect like planner for HIRO-NX
## 
## R.Hanai 2011.04.5 - 
##


import sys
import os
import time
import re
import operator

from numpy import *
from scipy import *
from scipy.spatial import *

from utils import *
from viewer import *
from mplan_env import *


class State:
    def __init__(self, parent=None, avec=zeros(7)):
        self.parent = parent
        self.cntrl = None
        self.avec = avec

class Node:
    def __init__(self, parent=None, q=zeros(7), x=FRAME()):
        self.parent = parent
        self.q = q
        self.x = x

class CSPlanner():
    def __init__(self, robot, env, cc=None):
        self.maxTime = 1.0
        self.robot = robot
        self.env = env
        self.cc = cc

        self.poses = []
        self.maxIter = 50
        self.epsilon = 60.0

        self.cccnt = 0
        self.cctm = 0.0


    def move_arm(self, goalrot, goalpos, width):
        """
        This is the explanation of the function, hoge.
        @rtc_service_signature: (in Mat33, in Vec3, in double) -> unsigned long
        """
        pass

    def grasp_plan(self, targetfrm, grasp_from_side=False,
                   offset=140, approach_distance=50):               
        if not grasp_from_side:
            gfrm1 = targetfrm*FRAME(xyzabc=[0,0,offset,0,-pi/2,0])
            afrm1 = targetfrm*FRAME(xyzabc=[0,0,offset+approach_distance,
                                            0,-pi/2,0])
            gfrm2 = gfrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
            afrm2 = afrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
            return [afrm1,afrm2],[gfrm1,gfrm2]
        else:
            gfrm1 = targetfrm*FRAME(xyzabc=[0,offset,0,-pi/2,-pi/2,0])
            afrm1 = targetfrm*FRAME(xyzabc=[0,offset+approach_distance,0,
                                            -pi/2,-pi/2,0])
            gfrm2 = gfrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
            afrm2 = afrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
            return [afrm1,afrm2],[gfrm1,gfrm2]

    def reaching_plan(self, objfrm, arm='right', use_waist=True,
                      target_type='box', offset=140, approach_distance=50,
                      grasp_from_side=False):
        afrms, gfrms = self.grasp_plan(objfrm, offset=offset,
                                       grasp_from_side=grasp_from_side,
                                       approach_distance=approach_distance)
        return self.robot.ik(afrms, arm, use_waist), self.robot.ik(gfrms, arm, use_waist)


    #def make_plan(self, q_start, p_targets):
    def make_plan(self, q_start, q_target):
        self.clean()
        self.T_init = [State(avec=array(q_start))]
        self.T_goal = [State(avec=array(q_target))]
        # self.T_goal = map(lambda x: State(avec=x), goal_configs)
        start_time = time.time()
        while time.time() - start_time < self.maxTime:
            if self.expandTree():
                traj = []
                nd = self.T_init[-1]
                while nd:
                    traj.append(nd)
                    nd = nd.parent
                traj.reverse()
                nd = self.T_goal[-1]
                while nd:
                    traj.append(nd)
                    nd = nd.parent

                return traj

        return None

    def optimize_trajectory(self, traj, tm=3.0):
        t1 = time.time()
        
        for i in range(len(traj)-1):
            traj[i].parent = traj[i+1]

        trajlen = len(traj)

        while True:
            t2 = time.time()
            if t2 - t1 > tm:
                break
            
            # choose 2 nodes randomly
            m = random.randint(0, trajlen-1)
            n = random.randint(0, trajlen-1)
            if m == n:
                continue
            if m > n:
                tmp = m
                m = n
                n = tmp

            print 'm,n=%d,%d'%(m,n) # try to connect 2 nodes

            q = traj[0]
            for j in range(trajlen):
                if j == m:
                    q_from = q
                if j == n:
                    q_to = q
                q = q.parent

            q0 = q_from
            
            while True:
                print self.dist(q_from, q_to)
                q_new = self.newState(q_from, q_to)
                if q_new == 'trapped':
                    break
                elif q_new == 'reached':
                    trajlen = trajlen - (n - m -1)
                    trajlen -= 1
                    while q_to != q0:
                        q = q_from.parent
                        q_from.parent = q_to
                        q_to = q_from
                        q_from = q
                        trajlen += 1
                    print 'new length = %d'%trajlen
                    break
                else:
                    q_new.parent = q_from
                    q_from = q_new

        traj2 = []
        q = traj[0]
        while q:
            traj2.append(q)
            q = q.parent
            
        return traj2

    def expandTree(self):
        T_a = self.T_init
        T_b = self.T_goal

        for i in range(self.maxIter):
            if i % 50 == 0:
                print 'iter=%d' % i
            q_rand = self.randomState()
            res1 = self.connect(T_a, q_rand)
            res2 = self.connect(T_b, q_rand)
            if res1 == 'reached' and res2 == 'reached':
                return True
        warn('failed')
        return False

    def extend(self, T, q_to):
        q_near = self.nearestNeighbor(q_to, T)
        q_new = self.newState(q_to, q_near)
        if q_new:
            T.append(q_new)
            q_new.parent = q_near
            # print ' dist:',dist(q_new, self.goal)
            # if distSO3R3(fk(q_new.avec), fk(self.goal.avec)) < 5e-2:

            if self.dist(q_new, q_to) < 5e-1:
                self.goal = q_new
                return 'reached'
            return 'advanced'
        return 'trapped'

    def clean(self):
        for pose in self.poses:
            pose.set_visible(False)
        self.poses = []

    def dist(self, q1, q2):
        return distRn(q1.avec, q2.avec)

    def connect(self, T, q_to):
        q_from = self.nearestNeighbor(q_to, T)
        while True:
            # print self.dist(q_from, q_to)
            q_new = self.newState(q_from, q_to)
            if q_new == 'trapped':
                return q_new
            if q_new == 'reached':
                return q_new
            if q_new:
                q_new.parent = q_from
                T.append(q_new)
                # print ' dist:',dist(q_new, self.goals[0])
                # if dist(q_new, self.goals[0]) < 5e-1:
                #     self.goals[0].parent = q_new
                #     self.states.append(self.goals[0])
                #     return 'reached'
                # else:
                #     q_from = q_new
                q_from = q_new

    def findFeasiblePath(self):
        good_paths = []
        for goal in self.goals:
            q_best, cost = self.evalApproachConfig(goal)
            good_paths.append((q_best, cost))
        return good_paths

    def randomState(self):
        # pseudo random number generator of Python uses Mersenne Twister
        return State(avec=array([random.uniform(self.robot.arm_jlimits[0][0],self.robot.arm_jlimits[0][1]),
                                 random.uniform(self.robot.arm_jlimits[1][0],self.robot.arm_jlimits[1][1]),
                                 random.uniform(self.robot.arm_jlimits[2][0],self.robot.arm_jlimits[2][1]),
                                 random.uniform(self.robot.arm_jlimits[3][0],self.robot.arm_jlimits[3][1]),
                                 random.uniform(self.robot.arm_jlimits[4][0],self.robot.arm_jlimits[4][1]),
                                 random.uniform(self.robot.arm_jlimits[5][0],self.robot.arm_jlimits[5][1])]))

    def nearestNeighbor(self, q_in, T):
        return min(T, key=lambda q: distRn(q.avec, q_in.avec))

    def newState(self, q_from, q_to):
        v = q_to.avec - q_from.avec
        dws = 0.0
        for i in range(len(q_from.avec)):
            dws += self.robot.sampling_weight[i] * abs(v[i])

        if dws < self.epsilon * 2.0:
            return 'reached'
        else:
            dvec = self.epsilon * v / dws
            st = State(avec = q_from.avec + dvec)
            if self.feasibleState(st):
                return st
            else:
                return 'trapped'

    def feasibleState(self, q):
        self.robot.set_arm_joint_angles(q.avec)
        t1 = time.time()
        cstate = self.robot.in_collision()
        t2 = time.time()
        self.cccnt += 1
        self.cctm += t2-t1
        # print 'CC: ', t2-t1

        if cstate:
            print 'collision: ',
            print self.robot.get_arm_joint_angles()
        return not cstate


class SamplingBasedPlanner:
    def __init__(self, arm, cc, plenv):
        self.maxIter = 400
        self.arm = arm
        self.cc = cc
        self.envobjs = []
        self.plenv = plenv

    def reset(self):
        self.nodes = []

    def addGraspedObject(self, obj, relfrm):
        self.graspedobj = obj

    def addCollisionObjects(self, objs):
        self.envobjs = self.envobjs + objs

    def draw(self, frm_or_nodes):
        if len(frm_or_nodes) == 0:
            return
        self.plenv.clear_markers()
        if isinstance(frm_or_nodes[0], Node):
            for nd in frm_or_nodes:
                self.plenv.add_marker(nd.x)
        elif isinstance(frm_or_nodes[0], FRAME):
            for frm in frm_or_nodes:
                self.plenv.add_marker(frm)



class TaskSpacePlanner(SamplingBasedPlanner):
    def __init__(self, arm, cc, plenv):
        SamplingBasedPlanner.__init__(self, arm, cc, plenv)
        self.goalbias = 0.3
        self.cntrllimit = 50.0
        self.goalradius = 10.0 # distance in SE(3)

    def make_arm_plan(self, q_start, goal1, goal2=None):
        '''argument can be given as a frame or a pair of rotation and position'''
        self.arm.set_joint_angles(q_start)
        if isinstance(goal1, FRAME):
            goal_frame = goal1
        else:
            goal_frame = FRAME(mat=goal1, vec=goal2)

        self.x_goal = goal_frame
        T = [Node(parent=None, q=self.arm.get_joint_angles(), x=self.arm.endcoords())]

        t1 = time.time()

        for i in range(self.maxIter):
            if i % 50 == 0:
                print 'iter = %d' % i
            if random.random() > self.goalbias:
                x_rand = self.randomFrame()
            else:
                x_rand = goal_frame
            res = self.extend(T, x_rand)
            # res = self.connect(T, x_rand)

            if res != 'trapped':
                if res == 'reached':
                    print 'REACHED'
                    t2 = time.time()
                    print 'took %f [sec]'%(t2-t1)
                    return T, self.findGoodPath(T)

        t2 = time.time()
        print 'took %f [sec]'%(t2-t1)

        return T, None

    def connect(self, T, x_rand):
        print 'connect'
        for i in range(10):
            res = self.extend(T, x_rand)
            if res != 'advanced':
                return res

    def extend(self, T, x_rand):
        nn = self.nearestNeighbor(x_rand, T)
        q_near = nn.q
        x_near = nn.x

        u = self.tscontrol(q_near, x_near, x_rand)
        q_new = self.newState(q_near, u)
        self.arm.crop_with_joint_angle_limits(q_new)

        self.arm.set_joint_angles(q_new)
        x_new = self.arm.endcoords()
        #x_new = self.newState2(x_near, x_rand)
        print 'x_new'
        print x_new
        # if distSE3(x_near, x_new) > 300.0:
        #     print 'SINGULAR'
        # print ''

        if self.isCollisionFree():
            T.append(Node(parent=nn, q=q_new, x=x_new))
            if distSE3(x_new, self.x_goal) < self.goalradius:
                return 'reached'
            return 'advanced'

        print 'collision'
        return 'trapped'

    def randomFrame(self):
        return sampleSE3()

    def nearestNeighbor(self, x, T):
        return min(T, key=lambda nd: distSE3(nd.x, x))

    def tscontrol(self, q_near, x_near, x_rand):
        # compute preferable control in the task space
        dx = x_rand.vec - x_near.vec
        dm = x_rand.mat * (-x_near.mat)
        th,ax = dm.rot_axis()
        dw = th*ax
        # dw = VECTOR(x=0,y=0,z=0) # position only bias
        u = array(dx[0:3] + dw[0:3])
        u = self.cropWithinLimits(u)

        J = self.computeJacobi(q_near)

        if False:
            # pseudo inverse approach
            #
            # scipy.linalg.pinv: the (Moore-Penrose) pseudo-inverse
            # pinv(Least square), pinv2(singular value decomposition)
            #
            # >>> a = random.randn(9, 6)
            # >>> B = linalg.pinv(a)
            # >>> allclose(a, dot(a, dot(B, a)))
            # >>> allclose(B, dot(B, dot(a, B)))
            qdotref = self.secondaryControl(q_near, x_rand)
            Jdag = linalg.pinv(J)
            alpha = 0.0
            qdot = dot(Jdag,u) + alpha*(dot(identity(len(qdotref))-dot(Jdag,J),qdotref))
        else:
            # dumped least square approach
            JT = transpose(J)
            eps = 0.1
            qdot = dot(JT, dot(linalg.inv(dot(J,JT) + eps*identity(J.shape[0])), u))

        print 'q_near: ', q_near
        # print 'J: ', J
        print 'x_rand: ', x_rand
        print 'x_near: ', x_near
        print 'u: ', u
        print 'qdot: ', qdot

        dq = qdot
        return dq

    def cropWithinLimits(self, u):
        l = linalg.norm(u)
        if l > self.cntrllimit:
            return (self.cntrllimit / l) * u
        else:
            return u

    def computeJacobi(self, q_near):
        self.arm.set_joint_angles(q_near)
        R,x,J = self.arm.fk(compute_jacobi=True)
        return J

    def secondaryControl(self, q_near, x_rand):
        return zeros(7) # control in the nullspace

    def newState(self, q_near, u):
        return q_near + u

    def newState2(self, x_near, x_rand):
        epsilon = 50.0
        r_near = x_near.mat
        p_near = x_near.vec
        r_rand = x_rand.mat
        p_rand = x_rand.vec
        v = p_rand - p_near
        l = linalg.norm(v)
        if l < epsilon:
            return None
        else:
            dvec = v / l
            p_new = p_near + dvec * epsilon
            r_new = r_rand
            return FRAME(mat=r_new, vec=p_new)

    def findGoodPath(self, T):
        qs = []
        xs = []
        nd = T[-1]
        while nd:
            qs.append(nd.q)
            xs.append(nd.x)
            nd = nd.parent
        qs.reverse()
        xs.reverse()
        return qs,xs


class CspacePlanner(SamplingBasedPlanner):
    def __init__(self):
        self.epsilon = 0.15

    def calcGoalConfigs(self, efframe):
        # joint S2 samples
        s2samples = map(radians, [-40,-20,0,20,40])
        q_goals = reduce(operator.add,
                         map(lambda ths2: self.arm.ik(efframe, ths2),
                             s2samples))
        return q_goals

    def expandTree(self):
        T_a = self.T_init
        T_b = self.T_goal

        for i in range(self.maxIter):
            if i % 50 == 0:
                print 'iter=%d' % i
            q_rand = self.randomState()
            if self.extend(T_a, q_rand) != 'trapped':
                if self.extend(T_b, q_new) == 'reached':
                    return True
            tmp = T_a
            T_a = T_b
            T_b = T_a
        return False

    def extend(self, T, q_to):
        q_near = self.nearestNeighbor(q_to, T)
        q_new = self.newState(q_to, q_near)
        if q_new:
            T.append(q_new)
            q_new.parent = q_near
            # print ' dist:',dist(q_new, self.goal)
            # if distSO3R3(fk(q_new.avec), fk(self.goal.avec)) < 5e-2:

            if dist(q_new, q_to) < 5e-1:
                self.goal = q_new
                return 'reached'
            return 'advanced'
        return 'trapped'

    def makeArmPlan(self, goal_configs):
        self.T_init = [State(avec=array(self.arm.get_joints()))]
        self.T_goal = map(lambda x: State(avec=x), goal_configs)
        self.clean()
        self.expandTree()

    def clean(self):
        for pose in self.poses:
            pose.set_visible(False)
        self.poses = []
        # self.states = []

    def connect(self, q_to):
        q_from = self.nearestNeighbor(q_to)
        while True:
            q_new = self.newState(q_from, q_to)
            if q_new:
                q_new.parent = q_from
                self.states.append(q_new)
                # print ' dist:',dist(q_new, self.goals[0])
                # if dist(q_new, self.goals[0]) < 5e-1:
                #     self.goals[0].parent = q_new
                #     self.states.append(self.goals[0])
                #     return 'reached'
                # else:
                #     q_from = q_new
                q_from = q_new
            else:
                return 'trapped'

    def findFeasiblePath(self):
        good_paths = []
        for goal in self.goals:
            q_best, cost = self.evalApproachConfig(goal)
            good_paths.append((q_best, cost))
        return good_paths

    # def evalApproachConfig(self, goal):
    #     dmin = sys.float_info.max
    #     for q in self.states:
    #         # d = distSO3R3(r.fk(q.avec), r.fk(goal.avec))
    #         # d = scipy.spatial.distance.minkowski(q.avec, goal.avec, 2)
    #         d = 0.0
    #         i = 0
    #         while i < 6:
    #             d = d + (q.avec[i]-goal.avec[i])**2
    #             i = i + 1
    #         if d < dmin:
    #             dmin = d
    #             q_best = q
    #     return q_best, dmin

    def randomState(self):
        return sampleUniformState()

    def nearestNeighbor(self, q_in, T):
        return min(T, key=lambda q: dist(q, q_in))

    def newState(self, q_from, q_to):
        v = q_to.avec - q_from.avec
        l = linalg.norm(v)
        if l < 8e-1:
            return None
        else:
            dvec = v / l
            st = State(avec = q_from.avec + dvec * self.epsilon)
            if self.feasibleState(st):
                return st
            else:
                return None

    def feasibleState(self, q): # check collision
        return True

