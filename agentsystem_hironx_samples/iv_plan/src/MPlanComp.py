#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time


## ros dependent
import roslib; roslib.load_manifest('iv_plan')
##

import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist

from utils import *
from viewer import *
import scene_objects
from robot import *
from mplan_env import *
from csplan import *
import hironx_motions

real_robot = False
if real_robot:
    from real_hiro import *
    import rospy
    rr = RealHIRO()
else:
    rr = None

env = MPlanEnv()
env.load_scene(scene_objects.ac_scene())
r = VHIRONX(pkgdir+'/externals/models/HIRO_110603/')
env.insert_object(r, FRAME(), env.get_world())
r.go_pos(-150, 0, 0)
pl = CSPlanner(r, env)
r.add_collision_object(env.get_object('table top'))

def move_arm(goal,width,arm,use_torso,check_collision,duration=2.0):
    ori = goal.orientation
    pos = goal.position
    frm = FRAME(xyzabc=[pos.x,pos.y,pos.z,ori.r,ori.p,ori.y])
    try:
        sol = r.ik(frm, use_waist=use_torso, arm=arm)[0]
        if use_torso:
            r.set_joint_angle(0, sol[0])
            r.set_arm_joint_angles(sol[1], arm=arm)
        else:
            r.set_arm_joint_angles(sol, arm=arm)
        sync(duration=duration)
        return True
    except:
        return False

def sync(duration=4.0, joints='all', wait=True, waitkey=True):
    '''synchronize the real robot with the model in "duration" [sec]'''
    if rr:
        js = r.get_joint_angles()
        if joints == 'torso':
            rr.send_goal([js[0:3],[],[],[],[]], duration, wait=wait)
        elif joints == 'rarm':
            rr.send_goal([[],js[3:9],[],[],[]], duration, wait=wait)
        elif joints == 'larm':
            rr.send_goal([[],[],js[9:15],[],[]], duration, wait=wait)
        elif joints == 'rhand':
            rr.send_goal([[],[],[],js[15:19],[]], duration, wait=wait)
        elif joints == 'lhand':
            rr.send_goal([[],[],[],[],js[19:23]], duration, wait=wait)
        elif joints == 'torso_rarm':
            rr.send_goal([js[0:3],js[3:9],[],[],[]], duration, wait=wait)
        elif joints == 'torso_larm':
            rr.send_goal([js[0:3],[],js[9:15],[],[]], duration, wait=wait)
        elif joints=='all':
            rr.send_goal([js[0:3],js[3:9],js[9:15],js[15:19],js[19:23]], duration, wait=wait)
        else:
            warn('unknown joints parameter: ' + joints)


# Module specification
mplanserviceprovider_spec = ["implementation_id", "MPlan",
                             "type_name",         "MPlan",
                             "description",       "Planner for HIRO-NX",
                             "version",           "0.1.0",
                             "vendor",            "Ryo Hanai",
                             "category",          "Planner",
                             "activity_type",     "EVENTDRIVEN",
                             "kind",              "DataFlowComponent",
                             "max_instance",      "0",
                             "language",          "Python",
                             "lang_type",         "script",
                             ""]


class MPlanServiceSVC_impl(_GlobalIDL__POA.ArmMotionService):
    def __init__(self):
        return
    
    def __del__(self):
        pass

    def MoveArm(self, goal, handWidth, arm, useTorso, checkCollision):
        print 'goal: ', goal
        print 'handWidth: ', handWidth
        print 'arm: ', arm
        print 'useTorso: ', useTorso
        print 'checkCollision: ', checkCollision
        return move_arm(goal,handWidth,arm,useTorso,checkCollision)

    def SetJointAngles(self, jointAngles):
        return r.set_joint_angles(jointAngles)

    # def ResetEnv(self):
    #     return setup_ac_scene()


class MPlanServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        # initialization of CORBA Port
        self._armPlanServicePort = OpenRTM_aist.CorbaPort("ArmPlanService")

        # initialization of Provider
        self._service0 = MPlanServiceSVC_impl()

        # Set service providers to Ports
        self._armPlanServicePort.registerProvider("service0",
                                                  "ArmPlanService",
                                                  self._service0)

        # Set CORBA Service Ports
        self.addPort(self._armPlanServicePort)

        return RTC.RTC_OK


def MPlanInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=mplanserviceprovider_spec)
    manager.registerFactory(profile,
                            MPlanServiceProvider,
                            OpenRTM_aist.Delete)
    return


def MPlanModuleInit(manager):
    MPlanInit(manager)
    # Create a component
    comp = manager.createComponent("MPlan")
    return


def main():
    # Initialize manager
    mgr = OpenRTM_aist.Manager.init(sys.argv)

    # Set module initialization proceduer
    # This procedure will be invoked in activateManager() function.
    mgr.setModuleInitProc(MPlanModuleInit)

    # Activate manager and register to naming service
    mgr.activateManager()

    # run the manager in blocking mode
    # runManager(False) is the default
    mgr.runManager()

    # If you want to run the manager in non-blocking mode, do like this
    # mgr.runManager(True)


if __name__ == "__main__":
    main()
