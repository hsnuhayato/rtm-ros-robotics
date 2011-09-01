#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time

from set_env import *

import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist

##
## Imports of RTM must be before those of vpython.
## Otherwise, SEGV occurs when sending a CORBA message for some reason.
##

import visual
from ivutils import *
from viewer import *
import scene_objects
from robot import *
from mplan_env import *
from csplan import *
import hironx_motions


from demo_common import *


# Module specification
mplanserviceprovider_spec = ["implementation_id", "ArmPlan",
                             "type_name",         "ArmPlan",
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


def Pose3DtoFRAME(pose):
    ori = pose.orientation
    pos = pose.position
    return FRAME(xyzabc=[pos.x,pos.y,pos.z,ori.r,ori.p,ori.y])

def FRAMEtoPose3D(frm):
    pos = RTC.Point3D(frm.vec[0], frm.vec[1], frm.vec[2])
    a,b,c = frm.mat.abc()
    ori = RTC.Orientation3D(a, b, c)
    return RTC.Pose3D(ori, pos)

import operator

def encode_FRAME(f):
    return reduce(operator.__add__, f.mat) + f.vec

def decode_FRAME(ds):
    return FRAME(mat=array(ds[0:9]).reshape(3,3).tolist(), vec=ds[9:])

def prefix(objectType):
    table = {1:'A', 2:'B'}
    try:
        return table[objectType]
    except:
        return []


class ArmPlanServiceSVC_impl(_GlobalIDL__POA.ArmMotionService):
    def __init__(self):
        return
    
    def __del__(self):
        pass

    def MoveArm(self, goal, handWidth, joints, checkCollision, duration):
        frm = decode_FRAME(goal)
        if handWidth <= 0.0: # doesn't move gripper
            handWidth = None
        return move_arm(frm,joints=joints, width=handWidth,
                        check_collision=checkCollision, duration=duration)
    
    def MoveArm2(self, afrm, gfrm, handWidth, joints):
        afrm = decode_FRAME(afrm)
        gfrm = decode_FRAME(gfrm)
        return move_arm2(afrm, gfrm, handWidth, joints=joints)

    def Grab(self, hand):
        return grab(hand=hand)

    def Release(self, hand):
        return release(hand=hand)

    def Move(self, goalConfig, joints):
        q0 = r.get_joint_angles(joints=joints)
        print q0
        print goalConfig
        traj = pl.make_plan(q0, goalConfig, joints=joints)
        exec_traj(traj, joints=joints)
        return True

    def MoveArmRelative(self, reltrans, handWidth, joints, checkCollision, duration):
        f = decode_FRAME(reltrans)
        g = r.fk(parse_joints_flag(joints)[0])
        frm = FRAME(mat=g.mat*f.mat, vec=g.vec+f.vec)
        if handWidth <= 0.0:
            handWidth = None
        return move_arm(frm,joints=joints, width=handWidth,
                        check_collision=checkCollision, duration=duration)

    def GoPreparePose(self):
        return go_prepare_pose()

    def GetJointAngles(self, joints):
        return r.get_joint_angles(joints=joints)

    def Fk(self, joints):
        return encode_FRAME(r.fk(parse_joints_flag(joints)[0]))

    def Ik(self, frm, joints):
        return r.ik(decode_FRAME(frm), joints=joints)[0]

    def GraspPlan(self, otype, ofrm, longSide):
        ofrm = decode_FRAME(ofrm)
        afrm,gfrm,handwidth = graspplan(otype, ofrm, long_side=longSide)
        return encode_FRAME(afrm) + encode_FRAME(gfrm) + [handwidth]

    def RequestNext(self, afrm, gfrm, handwidth):
        afrm2,gfrm2 = request_next(afrm, gfrm)
        return encode_FRAME(afrm2) + encode_FRAME(gfrm2) + [handwidth]

    def PlacePlan(self, otype, pfrm):
        pfrm = decode_FRAME(pfrm)
        afrm,gfrm = placeplan(otype, pfrm)
        return encode_FRAME(afrm) + encode_FRAME(gfrm)

    def RecognizePocket(self, objectType):
        def occupied(p, otyp):
            eps = 10.0
            return reduce(operator.__or__,
                          [linalg.norm(array(o.where().vec[:2]-array(p.where().vec[:2]))) < eps
                           for o in env.get_objects(prefix(objectType))])

        os = [o for o in env.get_objects('P') if not occupied(o, objectType)]
        if len(os) > 0:
            return encode_FRAME(os[0].where())
        else:
            return []

    def RecognizeParts(self, objectType):
        os = [o for o in env.get_objects(prefix(objectType)) if o.where().vec[1] > -80]

        if len(os) > 0:
            return encode_FRAME(os[0].where())
            #return encode_FRAME(os[random.randint(0,len(os)-1)].where())
        else:
            return []

    def ResetWorld(self):
        reset_parts()
        

class ArmPlanServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        # initialization of CORBA Port
        self._armPlanServicePort = OpenRTM_aist.CorbaPort("ArmPlanService")

        # initialization of Provider
        self._service0 = ArmPlanServiceSVC_impl()

        # Set service providers to Ports
        self._armPlanServicePort.registerProvider("service0",
                                                  "ArmPlanService",
                                                  self._service0)

        # Set CORBA Service Ports
        self.addPort(self._armPlanServicePort)

        return RTC.RTC_OK


def ArmPlanInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=mplanserviceprovider_spec)
    manager.registerFactory(profile,
                            ArmPlanServiceProvider,
                            OpenRTM_aist.Delete)
    return


def ArmPlanModuleInit(manager):
    ArmPlanInit(manager)
    # Create a component
    comp = manager.createComponent("ArmPlan")
    return


def main():
    # Initialize manager
    mgr = OpenRTM_aist.Manager.init(sys.argv)

    # Set module initialization proceduer
    # This procedure will be invoked in activateManager() function.
    mgr.setModuleInitProc(ArmPlanModuleInit)

    # Activate manager and register to naming service
    mgr.activateManager()

    # run the manager in blocking mode
    # runManager(False) is the default
    mgr.runManager()

    # If you want to run the manager in non-blocking mode, do like this
    # mgr.runManager(True)


if __name__ == "__main__":
    main()
