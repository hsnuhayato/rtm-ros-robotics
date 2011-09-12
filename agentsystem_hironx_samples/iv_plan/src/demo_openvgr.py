# -*- coding: utf-8 -*-

#import roslib; roslib.load_manifest('iv_plan')

from numpy import *

import optparse
import os.path
import sys
import threading
import time
import traceback
import rtctree.tree
import rtctree.utils

import OpenRTM_aist

from rtshell import comp_mgmt
from rtshell import modmgr
import rtshell.path
from rtshell import port_types
import rtprint_comp
#import rtshell

from set_env import *
import RTC_grx
from demo_common import *


prompt = 'continue? (y/n): '

def start_reading_port(raw_paths, options, tree=None):
    event = threading.Event()

    mm = modmgr.ModuleMgr(verbose=options.verbose, paths=options.paths)
    mm.load_mods_and_poas(options.modules)
    if options.verbose:
        print >>sys.stderr, \
                'Pre-loaded modules: {0}'.format(mm.loaded_mod_names)
    if options.timeout == -1:
        max = options.max
        if options.verbose:
            print >>sys.stderr, 'Will run {0} times.'.format(max)
    else:
        max = -1
        if options.verbose:
            print >>sys.stderr, 'Will stop after {0}s'.format(options.timeout)

    targets = port_types.parse_targets(raw_paths)
    if not tree:
        paths = [t[0] for t in targets]
        tree = rtctree.tree.RTCTree(paths=paths, filter=paths)
    port_specs = port_types.make_port_specs(targets, mm, tree)
    port_types.require_all_input(port_specs)
    if options.verbose:
        print >>sys.stderr, \
                'Port specifications: {0}'.format([str(p) for p in port_specs])

    comp_name, mgr = comp_mgmt.make_comp('iv_scenario', tree,
            rtprint_comp.Reader, port_specs, event=event, rate=options.rate,
            max=max)
    if options.verbose:
        print >>sys.stderr, 'Created component {0}'.format(comp_name)
    comp = comp_mgmt.find_comp_in_mgr(comp_name, mgr)
    comp_mgmt.connect(comp, port_specs, tree)
    comp_mgmt.activate(comp)

    return comp, mgr, tree
    try:
        if options.timeout != -1:
            event.wait(options.timeout)
            comp_mgmt.disconnect(comp)
            comp_mgmt.deactivate(comp)
        elif options.max > -1:
            event.wait()
            result = comp.get()
            comp_mgmt.disconnect(comp)
            comp_mgmt.deactivate(comp)
        else:
            while True:
                raw_input()
            # The manager will catch the Ctrl-C and shut down itself, so don't
            # disconnect/deactivate the component
    except KeyboardInterrupt:
        pass
    except EOFError:
        pass

    tree.give_away_orb()
    del tree
    comp_mgmt.shutdown(mgr)
    return result

#ports = ['Recognition0.rtc:RecognitionResultOut']
#ports = ['RobotHardware0.rtc:jointStt', 'Flip0.rtc:boxPose']
ports = ['Flip0.rtc:boxPose']

# main(argv=[pt])
# options = {'paths': [], 'verbose': False, 'max': -1, 'modules': [], 'rate': 100.0, 'timeout': -1}

options = optparse.Values()
options.paths = []
options.verbose = False
options.max = -1
options.modules = ['RTC_grx']
options.rate = 10.0
options.timeout = -1
tree = None

comp = None
mgr = None
tree = None

def main():
    global comp, mgr, tree
    comp,mgr,tree= start_reading_port([rtshell.path.cmd_path_to_full_path(p) for p in ports], options, tree)


def preapproach():
    f = r.fk()
    f.vec[2] += 50
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=2.5)

    r.prepare(width=80)
    f = FRAME(xyzabc=[200,-50,1000,0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync()

# def get_joint_angles():
#     return reduce(operator.__add__, comp.get()[0].qState)

def detect_pose3d(scl=1.0):
    pose3d = comp.get()[0]
    #pose3d = comp.get()[1]
    u = pose3d.position.x
    v = pose3d.position.y
    w = pose3d.position.z
    print 'w=', w
    R = pose3d.orientation.r
    P = pose3d.orientation.p
    Y = pose3d.orientation.y

    a = 182
    b = 134
    c = -6
    d = -2
    e = 216
    x = a / 640.0 * (u - 320) + c
    y = b / 480.0 * (v - 240) + d
    z =  e / w
    Tcam_obj = FRAME(xyzabc=[scl*x,scl*y,scl*z,R,P,-Y])
    print 'cam=>obj: ', Tcam_obj

    q = rr.get_joint_angles()
    r.set_joint_angles(q)
    Twld_cam = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam

    Twld_obj = Twld_cam * Tcam_obj

    m = array(Twld_obj.mat)
    a = cross(m[0:3,2], [0,0,1])
    m2 = MATRIX(angle=linalg.norm(a), axis=VECTOR(vec=a.tolist()))
    Twld_obj.mat = m2*Twld_obj.mat
    return Twld_obj

def pick(f, h = 723, dosync=True):
    f.vec[2] = h
    f2 = f * (-r.Twrist_ef)
    sols = r.ik(f2)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-r.Twrist_ef)
        sols = r.ik(f2)
    r.set_arm_joint_angles(sols[0])
    if dosync:
        sync(joints='rarm', duration=2.5)
        for w in [80,75,70,65,60,55,50,45,40,34]:
            r.grasp(w)
            sync(duration=0.3)

def transport():
    f = r.fk()
    f.vec[2] += 150
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=2.5)
    f = FRAME(xyzabc=[200,-300,1000,0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync(joints='rarm', duration=2.5)

def place(f, h = 746, dosync=True):
    f.vec[2] = h
    f2 = f * (-r.Twrist_ef)
    sols = r.ik(f2)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-r.Twrist_ef)
        sols = r.ik(f2)
    r.set_arm_joint_angles(sols[0])
    if dosync:
        sync(joints='rarm', duration=2.5)
        for w in [40,50,60,70,80]:
            r.grasp(w)
            sync(duration=0.3)

def pick_and_place():
    f = detect_pose3d()
    pick(f)
    transport()
    time.sleep(2.5)
    f = detect_pose3d()
    place(f)
    preapproach()


# def main():
#     try:
#         val = read_from_ports([path.cmd_path_to_full_path(pt)], options, tree)
#         val = val.data
#         f = array(val[8:20])
#         f.resize([3,4])
#         print f[0:3,3]
#         Twld_obj = convert(f)
#         show_frame(Twld_obj)
#         key = raw_input(prompt)
#         if key != 'y':
#             return f
#         pickup(Twld_obj)
#     except Exception, e:
#         print >> sys.stderr, '{0}: {1}'.format(os.path.basename(sys.argv[0]), e)
#         return 1
#     return 0


# f = array([[  9.98929562e-01,   0.00000000e+00,   4.62572077e-02,
#               -1.50504138e+01],
#            [  3.20064736e-02,  -7.21970315e-01,  -6.91183369e-01,
#               -3.07515927e+01],
#            [  3.33963308e-02,   6.91924030e-01,  -7.21197491e-01,
#               7.43907387e+02]])

# def convert(f):
#     Tcam_obj = FRAME(mat=f[0:3,0:3].tolist(),vec=f[0:3,3].tolist())
#     Twld_cam = FRAME(xyzabc=[795,-170,1330,pi/2,-pi/2,0])*FRAME(mat=MATRIX(a=51.8*pi/180.0))
#     Twld_obj = Twld_cam*Tcam_obj
#     print 'world->obj: ', Twld_obj
#     return Twld_obj

# def pickup(f):
#     fa = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+202.0,0,-pi/2,0])
#     fg = FRAME(xyzabc=[f.vec[0],f.vec[1],f.vec[2]+152.0,0,-pi/2,0])
#     w,avec = r.ik(fa, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     r.grasp(width=100)
#     sync(duration=2.0)
#     key = raw_input(prompt)
#     if key != 'y':
#         return
#     w,avec = r.ik(fg, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     sync(duration=1.5)
#     r.grasp(width=24)
#     sync(duration=1.5)
#     fp = r.fk()
#     fp.vec[2] += 50
#     w,avec = r.ik(fp, use_waist=True)[0]
#     r.set_joint_angle(0, w)
#     r.set_arm_joint_angles(avec)
#     sync(joints='rarm', duration=1.5)
