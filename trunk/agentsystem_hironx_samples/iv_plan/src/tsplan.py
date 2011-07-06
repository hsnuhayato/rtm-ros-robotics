##
## task space sampling RRT planner
## 
## R.Hanai 2010.09.22 - 
##


import sys
import time
import os
from numpy import *
from scipy import *
from scipy.spatial import *

from subprocess import Popen, PIPE
import re
import operator

from set_env import *
from geo import *

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



class SamplingBasedPlanner:
    def __init__(self, arm, cc, plenv):
        self.maxIter = 400
        self.arm = arm
        self.cc = cc
        self.envobjs = []
        self.plenv = plenv

    def reset(self):
        self.nodes = []

    def isCollisionFree(self):
        if not (self.cc.isCollisionFree(self.arm.cbodies[0],self.arm.cbodies[2]) and
                self.cc.isCollisionFree(self.arm.cbodies[0],self.arm.cbodies[3]) and
                self.cc.isCollisionFree(self.arm.cbodies[1],self.arm.cbodies[3])):
            return False
                
        for envobj in self.envobjs:
            if not (self.cc.isCollisionFree(self.arm.cbodies[1],envobj) and
                    self.cc.isCollisionFree(self.arm.cbodies[2],envobj) and
                    self.cc.isCollisionFree(self.arm.cbodies[3],envobj)):
                return False
        
        # self collision test
        # return [self.cc.isCollisionFree(self.arm.cbodies[0],self.arm.cbodies[2]),
        #         self.cc.isCollisionFree(self.arm.cbodies[0],self.arm.cbodies[3]),
        #         self.cc.isCollisionFree(self.arm.cbodies[1],self.arm.cbodies[3])]

        # collision test with the environment
        # for envobj in self.envobjs:
        #     print self.cc.isCollisionFree(self.arm.cbodies[1],envobj)
        #     print self.cc.isCollisionFree(self.arm.cbodies[2],envobj)
        #     print self.cc.isCollisionFree(self.arm.cbodies[3],envobj)

        return True

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


