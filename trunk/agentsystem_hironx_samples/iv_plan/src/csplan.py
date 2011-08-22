##
## RRT-connect like planner
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


    # def grasp_plan(self, targetfrm, grasp_from_side=False,
    #                offset=140, approach_distance=50):               
    #     if not grasp_from_side:
    #         gfrm1 = targetfrm*FRAME(xyzabc=[0,0,offset,0,-pi/2,0])
    #         afrm1 = targetfrm*FRAME(xyzabc=[0,0,offset+approach_distance,
    #                                         0,-pi/2,0])
    #         gfrm2 = gfrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
    #         afrm2 = afrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
    #         return [afrm1,afrm2],[gfrm1,gfrm2]
    #     else:
    #         gfrm1 = targetfrm*FRAME(xyzabc=[0,offset,0,-pi/2,-pi/2,0])
    #         afrm1 = targetfrm*FRAME(xyzabc=[0,offset+approach_distance,0,
    #                                         -pi/2,-pi/2,0])
    #         gfrm2 = gfrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
    #         afrm2 = afrm1*FRAME(xyzabc=[0,0,0,pi,0,0])
    #         return [afrm1,afrm2],[gfrm1,gfrm2]

    # def reaching_plan(self, objfrm, arm='right', use_waist=True,
    #                   target_type='box', offset=140, approach_distance=50,
    #                   grasp_from_side=False):
    #     afrms, gfrms = self.grasp_plan(objfrm, offset=offset,
    #                                    grasp_from_side=grasp_from_side,
    #                                    approach_distance=approach_distance)
    #     return self.robot.ik(afrms, arm, use_waist), self.robot.ik(gfrms, arm, use_waist)


class State:
    def __init__(self, parent=None, avec=zeros(6)):
        self.parent = parent
        self.cntrl = None
        self.avec = avec

    # def __repr__(self):
    #     return '<%s %s>'%(self.__class__, self.avec)

    # def __str__(self):
    #     return self.__repr__()

class Node:
    def __init__(self, parent=None, q=zeros(6), x=FRAME()):
        self.parent = parent
        self.q = q
        self.x = x

def random_interval(trajlen):
    # choose 2 nodes randomly
    l = min(trajlen/2, 8)
    while True:
        m = random.randint(0, trajlen-1)
        n = random.randint(0, trajlen-1)
        if abs(m - n) < l:
            continue
        if m > n:
            tmp = m
            m = n
            n = tmp
        print 'm,n=%d,%d'%(m,n) # try to connect 2 nodes
        return m,n

def inflection_points(traj, thre=0.85):
    trajlen = len(traj)
    for i in range(trajlen-1):
        traj[i].parent = traj[i+1]

    dp = [traj[i].avec-traj[i+1].avec for i in range(len(traj)-1)]
    dp = [dp[i]/linalg.norm(dp[i]) for i in range(len(dp))]
    return [abs(dot(dp[i],dp[i+1])) < thre for i in range(len(dp)-1)]


class CSPlanner():
    def __init__(self, robot, env, cc=None):
        self.maxTime = 1.0
        self.robot = robot
        self.env = env
        self.cc = cc
        self.maxIter = 200
        self.epsilon = 60.0
        self.use_sampling_heuristics = False
        self.poses = []

    def initialize(self, joints='rarm'):
        # joints ::= rarm | larm | torso_rarm | torso_larm | all
        for pose in self.poses:
            pose.set_visible(False)
        self.poses = []
        self.cccnt = 0
        self.cctm = self.searchtm = self.optimizetm = 0.0

        self.joints = joints
        if joints == 'rarm':
            self.js = self.robot.joints[3:9]
        elif joints == 'larm':
            self.js = self.robot.joints[9:15]
        elif joints == 'torso_rarm':
            self.js = [self.robot.joints[0]]+self.robot.joints[3:9]
        elif joints == 'torso_larm':
            self.js = [self.robot.joints[0]]+self.robot.joints[9:15]
        else:
            self.js = self.robot.joints

        self.sllimits = array([j.sllimit for j in self.js])
        self.sulimits = array([j.sulimit for j in self.js])
        self.weights = array([j.weight for j in self.js])

    def random_state(self):
        return State(avec=numpy.random.random_sample(len(self.js))
                     * (self.sulimits-self.sllimits) + self.sllimits)

    def ws_sample_state(self):
        warn('not yet implemented')
        pass

    def print_statistics(self):
        colored_print('total time %f[sec]'%(self.searchtm + self.optimizetm), 'cyan')
        colored_print('search %f[sec]'%self.searchtm, 'cyan')
        colored_print('optimize %f[sec]'%self.optimizetm, 'cyan')
        colored_print('collision check %d[times], %f[sec]'%(self.cccnt, self.cctm), 'cyan')

    def make_plan(self, q_start, q_target, q_targets=[], joints='rarm'):
        self.initialize(joints=joints)
        traj = self.search(q_start, q_targets if len(q_targets)>0 else [q_target])
        if traj:
            opttraj = self.optimize_trajectory(traj)
            result = (traj, opttraj)
        else:
            result = None
        self.print_statistics()
        return result
    
    def search(self, q_start, q_targets):
        self.T_init = [State(avec=array(q_start))]
        self.T_goal = [State(avec=array(q)) for q in q_targets]

        tm_start = time.time()
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

            self.searchtm = time.time() - tm_start
            return traj
        return None

    def expandTree(self):
        T_a = self.T_init
        T_b = self.T_goal

        for i in range(self.maxIter):
            if self.use_sampling_heuristics:
                q_rand = self.ws_sample_state()
            else:
                q_rand = self.random_state()
            if i % 3 == 0:
                if self.extend(T_a, q_rand) != 'trapped':
                    q_new = T_a[-1]
                    if self.connect(T_b, q_new) == 'reached':
                        return True
            else:
                self.connect(T_a, q_rand, maxIter=5)

            T_tmp = T_a
            T_a = T_b
            T_b = T_tmp
        
        return False

    def extend(self, T, q_to):
        q_near = self.nearestNeighbor(q_to, T)
        q_new, dws = self.newState(q_near, q_to)
        if q_new == 'trapped':
            return q_new
        else:
            q_new.parent = q_near
            T.append(q_new)
            if dws < self.epsilon * 1.5:
                return 'reached'
            else:
                return 'advanced'

    def connect(self, T, q_to, maxIter=100):
        state = 'advanced'
        i = 0
        while state == 'advanced':
            # print self.dist(q_from, q_to)
            state = self.extend(T, q_to)
            if i == maxIter:
                break
            i += 1
        return state

    def nearestNeighbor(self, q_in, T):
        #return min(T, key=lambda q: distRn(q.avec, q_in.avec))
        return min(T, key=lambda q: self.weightedDistance(q.avec, q_in.avec))

    def dist(self, q1, q2):
        return distRn(q1.avec, q2.avec)

    def weightedDistance(self, q1, q2):
        return self.weightedNorm(q1 - q2)

    def weightedNorm(self, dq):
        dws = 0.0
        if self.joints == 'all':
            dws1 = 0.0
            for i in range(3,9):
                dws1 += self.weights[i] * abs(dq[i])
            dws2 = 0.0
            for i in range(9,15):
                dws2 += self.weights[i] * abs(dq[i])
            dws = max(dws1, dws2) + self.weights[0] * abs(dq[0])
        else:
            for i in range(len(dq)):
                dws += self.weights[i] * abs(dq[i])
        return dws

    def newState(self, q_near, q_to):
        v = q_to.avec - q_near.avec
        dws = self.weightedNorm(v)

        dvec = self.epsilon * v / dws
        st = State(avec = q_near.avec + dvec)
        if self.feasibleState(st):
            return st, dws
        else:
            return 'trapped', dws

    def feasibleState(self, q):
        for i in range(len(q.avec)):
            self.js[i].angle = q.avec[i]
        self.robot.refresh()

        t1 = time.time()
        cstate = self.robot.in_collision()
        t2 = time.time()
        self.cccnt += 1
        self.cctm += t2-t1

        # if cstate:
        #     print 'collision: ',
        #     print self.robot.get_arm_joint_angles()
        return not cstate

    def optimize_trajectory(self, traj):
        t1 = time.time()
        traj = self.smooth(self.random_shortcut(traj, tm=2.0))
        self.optimizetm = time.time() - t1
        return traj

    def random_shortcut(self, traj, tm=3.0):
        t1 = time.time()
        trajlen = len(traj)        
        for i in range(trajlen-1):
            traj[i].parent = traj[i+1]

        while True:
            t2 = time.time()
            if t2 - t1 > tm:
                break
            m,n = random_interval(trajlen)
            trajlen = self.try_shortcut(traj, m, n, trajlen)

        traj2 = []
        q = traj[0]
        while q:
            traj2.append(q)
            q = q.parent

        return traj2

    def try_shortcut(self, traj, m, n, trajlen):
        q = traj[0]
        for j in range(trajlen):
            if j == m:
                q_from = q
            if j == n:
                q_to = q
            q = q.parent

        q0 = q_from
            
        while True:
            q_new, dws = self.newState(q_from, q_to)
            if q_new == 'trapped':
                return trajlen
            else:
                if dws < self.epsilon * 1.5:
                    trajlen = trajlen - (n - m -1)
                    trajlen -= 1
                    while q_to != q0:
                        q = q_from.parent
                        q_from.parent = q_to
                        q_to = q_from
                        q_from = q
                        trajlen += 1
                    print 'new length = %d'%trajlen
                    return trajlen
                else:
                    q_new.parent = q_from
                    q_from = q_new

    def smooth(self, traj):
        a = 3
        trajlen = len(traj)
        for k in range(5):
            is_shortened = False
            ifpts = inflection_points(traj)
            i = len(ifpts)-2
            while i >= a:
                if ifpts[i]:
                    m = i-a
                    n = i+a
                    #colored_print('k,m,n=%d,%d,%d'%(k,m,n), 'red')
                    newtrajlen = self.try_shortcut(traj, m, n, trajlen)
                    if newtrajlen < trajlen:
                        is_shortened = True                    
                        i -= a
                    else:
                        i -= 1
                    trajlen = newtrajlen
                else:
                    i -= 1

            traj2 = []
            q = traj[0]
            while q:
                traj2.append(q)
                q = q.parent
            traj = traj2
            
            if not is_shortened:
                break

        return traj


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
