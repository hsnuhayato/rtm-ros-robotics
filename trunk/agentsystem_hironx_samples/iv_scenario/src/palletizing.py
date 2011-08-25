##
## scenario.py
## R.Hanai 2011.05.23 -
##

import roslib; roslib.load_manifest('iv_plan')

from rtc_handle import *
import RTC
import _GlobalIDL
import rospy
import math
from numpy import *
from geo import *

def get_handle(name, nspace):
    return nspace.rtc_handles[name]

def narrow_service(handle, service, port):
    svc = handle.services[service].provided[port]
    svc.narrow_ref(globals())
    return svc

def activate(handles):
    for hdl in handles:
        hdl.activate()

def deactivate(handles):
    for hdl in handles:
        hdl.deactivate()

def connect(handle1, port1, handle2, port2):
    con = IOConnector([handle1.outports[port1], handle2.inports[port2]])
    con.connect()

def disconnect(con):
    con.disconnect()


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


nameserver = 'hiro014:2809'
env = RtmEnv(sys.argv, [nameserver])
ns = env.name_space[nameserver]
ns.list_obj()

hpl = get_handle('MPlan0.rtc', ns)
hpl.activate()
plsvc = hpl.services['ArmPlanService'].provided['service0']


# objType definition:
# 1: parts A
# 2: parts B
# 3: pallet pocket P

import re

def approach_test(objType = 1, joints='rarm'):
    ofrm = plsvc.ref.RecognizeParts(objType)
    a = plsvc.ref.GraspPlan(objType, ofrm, False)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        raw_input('Press any key to reset pose: ')
        plsvc.ref.MoveArm(afrm, handwidth, joints, False, 0.5)
        plsvc.ref.GoPreparePose()
    else:
        raw_input('Cannot reach with %s !' % joints)

def pick_and_place(ofrm, objType = 1, joints='torso_rarm'):
    hand = 'right' if re.sub('torso_', '', joints) == 'rarm' else 'left'
    a = plsvc.ref.GraspPlan(objType, ofrm, False)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        a = plsvc.ref.RequestNext(afrm, gfrm, handwidth) # try next grasp
        afrm = a[:12]
        gfrm = a[12:24]
        handwidth = a[24]
        if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
            print 'Cannot reach with %s !' % joints
            return False

    plsvc.ref.Grab(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)

    pfrm = plsvc.ref.RecognizePocket(objType) # find a pocket where grasped parts can be put
    a = plsvc.ref.PlacePlan(objType, pfrm)
    afrm = a[:12]
    gfrm = a[12:24]
    plsvc.ref.MoveArm2(afrm, gfrm, handwidth+20, joints)

    plsvc.ref.Release(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)

def palletize_right():
    def aux(objType):
        while True:
            ofrm = plsvc.ref.RecognizeParts(objType)
            if not ofrm:
                return
            pick_and_place(ofrm, objType)

    aux(1); aux(2)
    plsvc.ref.GoPreparePose()

def pass_left_to_right(objType):
    if objType == 1:
        q_goal = [0, 0.18962052706991084, -0.30309547203097081, -1.6161104682040897,
                  -1.1672811346866405, -0.12695972539338643, -0.23692180236594873,
                  -0.13308174660428049, -0.36125317532098755, -1.5033910600950069,
                  1.3706292369222353, 0.0028576290419997661, 1.7279863708909555]
        handwidth = 38
    else:
        q_goal = [0, 0.10953652101989117, -0.32418125972003026, -1.619220151757599,
                  -1.1960786166203574, -0.048452045171419739, -0.23084568842323364,
                  -0.13308174660428049, -0.36125317532098755, -1.5033910600950069,
                  1.3706292369222353, 0.0028576290419997661, 1.7279863708909555]
        handwidth = 25

    plsvc.ref.Move(q_goal, 'torso_arms') # whole body motion
    plsvc.ref.MoveArmRelative(encode_FRAME(FRAME()), handwidth, 'rarm', False, 0.5) # just close the hand
    plsvc.ref.Release('left')
    plsvc.ref.Grab('right')
    plsvc.ref.MoveArmRelative(encode_FRAME(FRAME()), 100, 'larm', False, 0.5)
    plsvc.ref.MoveArmRelative(encode_FRAME(FRAME(xyzabc=[0,-50,0,0,0,0])), -1, 'rarm', False, 0.5)

def pick_pass_and_place(ofrm, objType):
    plsvc.ref.GoPreparePose()
    joints = 'torso_larm'
    hand = 'left'
    a = plsvc.ref.GraspPlan(objType, ofrm, True)
    afrm = a[:12]
    gfrm = a[12:24]
    handwidth = a[24]
    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
        a = plsvc.ref.RequestNext(afrm, gfrm, handwidth) # try next grasp
        afrm = a[:12]
        gfrm = a[12:24]
        handwidth = a[24]
        if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth, joints):
            print 'Cannot reach with %s' % joints
            return False

    plsvc.ref.Grab('left')
    pass_left_to_right(objType)

    joints = 'torso_rarm'
    hand = 'right'
    pfrm = plsvc.ref.RecognizePocket(objType)
    a = plsvc.ref.PlacePlan(objType, pfrm)
    afrm = a[:12]
    gfrm = a[12:24]

    if not plsvc.ref.MoveArm2(afrm, gfrm, handwidth+20, joints):
        print 'Cannot reach with %s' % joints
        return False
    
    plsvc.ref.Release(hand)
    plsvc.ref.MoveArm(afrm, -1, joints, False, 0.5)


def palletize():
    def aux(objType):
        while True:
            ofrm = plsvc.ref.RecognizeParts(objType)
            if not ofrm:
                return
            if decode_FRAME(ofrm).vec[1] < 50:
                pick_and_place(ofrm, objType)
            else:
                pick_pass_and_place(ofrm, objType)

    aux(1); aux(2)
    plsvc.ref.GoPreparePose()
    
