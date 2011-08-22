# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def approach(partsname, joints='rarm'):
    '''make a plan to a parts or a place from prepare position'''
    prepare() # initial pose
    r.set_joint_angle(0, 0.6) # turn the waist
    p1 = env.get_object(partsname).where() # get a goal frame of the end-effector
    p1.vec[2] += 20 # approach frame
    p1 = p1 * (-r.Twrist_ef) # calc a corresponding wrist frame
    q0 = r.get_joint_angles(joints=joints)
    q1 = r.ik(p1, joints=joints)[0]
    traj = pl.make_plan(q0, q1, joints=joints)
    if traj:
        show_traj(traj[1], joints)
        return traj

def plan(name1='A0', name2='P0', joints='rarm'):
    def get_config(name):
        p = env.get_object(name).where()
        p.vec[2] += 20
        return r.ik(p*(-r.Twrist_ef), joints=joints)[0]

    prepare()
    r.set_joint_angle(0, 0.6)
    traj = pl.make_plan(get_config(name1), get_config(name2), joints=joints)
    if traj:
        show_traj(traj[1], joints=joints)
        return traj

def graspplan(parts, long_side=False):
    ofrm = parts.where()
    if long_side:
        ofrm = ofrm * FRAME(xyzabc=[0,0,0,0,0,pi/2])
    gfrm = ofrm*(-r.Twrist_ef)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    if long_side:
        handwidth = parts.vbody.size[0]
    else:
        handwidth = parts.vbody.size[1]
    return afrm,gfrm,handwidth

def placeplan(place):
    gfrm = place.where()*(-r.Twrist_ef)
    gfrm.vec[2] += 20
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    return afrm,gfrm

def palletize(task_sequence=[], joints='rarm'):
    # parts: A0,A1,A2,A3, places: P0,P1,P2,P3
    if task_sequence == []:
        task_sequence = [('A0','P0'),('A1','P1'),('A2','P2'),('A3','P3')]
    
    prepare()
    r.set_joint_angle(0, 0.6)
    p0 = r.fk()

    jts = joints
    for oname,pname in task_sequence:
        l_or_r = 'right' if re.match('.*rarm$', jts) else 'left'
        parts = env.get_object(name=oname)
        afrm,gfrm,handwidth = graspplan(parts)
        move_arm(afrm, joints=jts, check_collision=True)
        move_arm(gfrm, joints=jts, width=handwidth, check_collision=False, duration=0.2)
        affix(parts, hand=l_or_r)
        move_arm(afrm, joints=jts, check_collision=False, duration=0.2)

        # for collision check
        # get object references from the environment using regular expression
        r.grasp_collision_object(parts, hand=l_or_r)
        for obj in env.get_objects('table top|pallete side|A'):
            if obj.name != parts.name:
                r.add_collision_pair(obj, parts)
        
        place = env.get_object(name=pname)
        afrm,gfrm = placeplan(place)
        move_arm(afrm, joints=jts, check_collision=True)
        move_arm(gfrm, joints=jts, width=handwidth+20, check_collision=False, duration=0.2)
        unfix(parts, hand=l_or_r)
        move_arm(afrm, joints=jts, check_collision=False, duration=0.2)

        # for collision check
        r.release_collision_object(parts, hand=l_or_r)
        for obj in env.get_objects('table top|pallete side|A'):
            if obj.name != parts.name:
                r.remove_collision_pair(obj, parts)

    move_arm(p0, joints=jts, check_collision=True)

def shift_right_to_left():
    prepare()
    r.set_joint_angle(0, 0.6)

    jts = 'torso_rarm'
    first = 'right'
    second = 'left'
    parts = env.get_object(name='A1')

    # remember the initial position
    afrm0,gfrm0,handwidth0 = graspplan(parts, long_side=True)

    afrm,gfrm,handwidth = graspplan(parts)
    move_arm(afrm, joints=jts, check_collision=True)
    move_arm(gfrm, joints=jts, width=handwidth, check_collision=False, duration=0.2)
    affix(parts, hand=first)
    move_arm(afrm, joints=jts, check_collision=False, duration=0.2)
    
    q0 = r.get_joint_angles()
    r.set_joint_angle(0, 0.0)
    fr = FRAME(mat=[[-0.0755156,-0.155534,-0.984940],
                    [-0.994845, 0.0787985, 0.0638317],
                    [0.0676838, 0.984683, -0.160683]],
               vec=[254.071, -95.291, 1013])
    fl = FRAME(mat=[[0.0756703, -0.984940, -0.155458],
                    [0.994765, 0.0638311, 0.0797926],
                    [-0.0686679, -0.160680, 0.984614]],
               vec=[264.071, 100, 1000])
    r.set_joint_angles(r.ik(fr, joints='rarm')[0], joints='rarm')
    r.set_joint_angles(r.ik(fl, joints='larm')[0], joints='larm')
    q1 = r.get_joint_angles(joints='all')
    traj = pl.make_plan(q0, q1, joints='all')
    if traj:
        show_traj(traj[1], joints='all')

    afrm,gfrm,handwidth = graspplan(parts, long_side=True)
    move_arm(fl, joints='larm', width=handwidth, check_collision=False, duration=0.0)
    unfix(parts, hand=first)
    affix(parts, hand=second)
    r.release_collision_object(parts, hand=first)
    for obj in env.get_objects('table top|pallete side|A'):
        if obj.name != parts.name:
            r.remove_collision_pair(obj, parts)
    r.grasp_collision_object(parts, hand=second)
    for obj in env.get_objects('table top|pallete side|A'):
        if obj.name != parts.name:
            r.add_collision_pair(obj, parts)

    jts = 'torso_larm'
    fl.vec[1] += 50
    move_arm(fl, joints='larm', check_collision=False, duration=0.2)
    move_arm(afrm0, joints=jts, check_collision=True)
    move_arm(gfrm0, joints=jts, width=handwidth+20, check_collision=False, duration=0.2)
    unfix(parts, hand=second)
    move_arm(afrm0, joints=jts, check_collision=False, duration=0.2)
    r.release_collision_object(parts, hand=second)
    for obj in env.get_objects('table top|pallete side|A'):
        if obj.name != parts.name:
            r.remove_collision_pair(obj, parts)

    q0 = r.get_joint_angles(joints='all')
    prepare()
    q1 = r.get_joint_angles(joints='all')
    traj = pl.make_plan(q0, q1, joints='all')
    if traj:
        show_traj(traj[1], joints='all')


# def pick_with_handcam():
#     prepare()
#     sync()
#     f = FRAME(xyzabc=[250,-50,850,0,0,0])
#     move_arm_ef(f)
#     Trhandcam_tgt = FRAME(xyzabc=[0,0,220,pi,0,pi/6]) # replaced with recognition result
#     Twld_tgt = r.get_sensor('rhandcam').where()*Trhandcam_tgt
#     move_arm_ef(Twld_tgt, duration=4.0)

def reset_parts():
    for i,pose in enumerate([[-260,-50,714,0,0,pi/6],
                             [-170,100,714,0,0,-pi/6],
                             [-250,190,714,0,0,0],
                             [-120,-10,714,0,0,pi/4]]):
        o = env.get_object('A'+str(i))
        o.unfix()
        o.affix(env.get_object('table'), FRAME(xyzabc=pose))


colored_print("approach('A0')", 'blue')
colored_print("approach('P0')", 'blue')
colored_print("approach('A2', joints='torso_larm')", 'blue')
colored_print("plan('A2','P3')", 'blue')
colored_print("plan('P1','A3', joints='torso_rarm')", 'blue')
colored_print("shift_right_to_left()", 'blue')
colored_print("palletize()", 'blue')
colored_print("reset_parts()", 'blue')
colored_print("palletize(task_sequence=[('A1','P1'),('A2','P2')]", 'blue')
