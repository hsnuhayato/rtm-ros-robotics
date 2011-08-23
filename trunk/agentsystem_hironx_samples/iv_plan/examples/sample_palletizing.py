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
        exec_traj(traj, joints=joints)
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
        exec_traj(traj, joints=joints)
        return traj

def graspplan(parts, long_side=False):
    ofrm = parts.where()
    if re.match('^A.*', parts.name):
        if long_side:
            ofrm = ofrm * FRAME(xyzabc=[0,0,0,0,0,pi/2])
            handwidth = parts.vbody.size[0]
        else:
            handwidth = parts.vbody.size[1]
        gfrm = ofrm*(-r.Twrist_ef)
        afrm = FRAME(gfrm)
        afrm.vec[2] += 40
    else:
        handwidth = parts.vbody.radius * 2
        ofrm = ofrm * FRAME(vec=[0,0,parts.vbody.axis[2]-15])
        gfrm = ofrm*(-r.Twrist_ef)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    
    return afrm,gfrm,handwidth

def request_next(afrm, gfrm):
    return afrm*FRAME(xyzabc=[0,0,0,pi,0,0]), gfrm*FRAME(xyzabc=[0,0,0,pi,0,0])

def placeplan(place, parts):
    gfrm = place.where()*(-r.Twrist_ef)
    if re.match('^A.*', parts.name):
        gfrm.vec[2] += parts.vbody.size[2]/2
    else:
        gfrm.vec[2] += (parts.vbody.axis[2]-15)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    return afrm,gfrm


def palletize(task_sequence=[]):
    # parts: A0,A1,A2,A3,B0,B1
    # places: P0,P1,P2,P3    
    if task_sequence == []:
        task_sequence = [('A0','P0','right'),('A2','P2','left'),('A1','P1','right'),
                         ('B0','P0','left'),('A3','P3','right'),('B1','P3','right')]

    prepare()
    p0 = r.fk()

    jts = 'torso_rarm'
    for oname,pname,l_or_r in task_sequence:
        if l_or_r == 'right':
            pick_and_place(oname, pname, joints=jts)
        else:
            move_arm(p0, joints=jts, check_collision=True)            
            pick_pass_and_place(oname, pname)

    go_prepare_pose()
    
def pick_and_place(oname='A0', pname='P0', joints='rarm'):
    jts = joints
    l_or_r,_ = parse_joints_flag(joints)
    parts = env.get_object(name=oname)
    afrm,gfrm,handwidth = graspplan(parts)
    if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
        afrm,gfrm = request_next(afrm, gfrm)
        if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
            return False
        
    affix(parts, hand=l_or_r)
    move_arm(afrm, joints=jts, check_collision=False, duration=0.2)
    # for collision check
    # get object references from the environment using regular expression
    r.grasp_collision_object(parts, hand=l_or_r)
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.add_collision_pair(obj, parts)
        
    place = env.get_object(name=pname)
    afrm,gfrm = placeplan(place, parts)
    if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
        afrm,gfrm = request_next(afrm, gfrm)
        if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
            return False

    unfix(parts, hand=l_or_r)
    move_arm(afrm, joints=jts, check_collision=False, duration=0.2)
    # for collision check
    r.release_collision_object(parts, hand=l_or_r)
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.remove_collision_pair(obj, parts)


def pass_left_to_right(parts, handwidth):
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
    if re.match('^B.*', parts.name):
        fr.vec[1] -= 26
        
    r.set_joint_angles(r.ik(fr, joints='rarm')[0], joints='rarm')
    r.set_joint_angles(r.ik(fl, joints='larm')[0], joints='larm')
    q1 = r.get_joint_angles(joints='all')
    traj = pl.make_plan(q0, q1, joints='all')
    exec_traj(traj, joints='all')

    move_arm(fr, joints='rarm', width=handwidth, check_collision=False, duration=0.2)
    unfix(parts, hand='left')
    affix(parts, hand='right')
    r.release_collision_object(parts, hand='left')
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.remove_collision_pair(obj, parts)
    r.grasp_collision_object(parts, hand='right')
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.add_collision_pair(obj, parts)
    fr.vec[1] -= 50
    move_arm(fr, joints='rarm', check_collision=False, duration=0.2)

def pick_pass_and_place(name1='A1', name2='P0'):
    # 
    jts = 'torso_larm'
    first = 'left'
    second = 'right'
    parts = env.get_object(name=name1)

    # pick 'A1' with 'lhand'
    afrm,gfrm,handwidth = graspplan(parts, long_side=True)
    if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
        afrm,gfrm = request_next(afrm, gfrm)
        if not move_arm2(afrm, gfrm, joints=jts, width=handwidth):
            return False

    affix(parts, hand=first)
    move_arm(afrm, joints=jts, check_collision=False, duration=0.2)
    r.grasp_collision_object(parts, hand=first)
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.add_collision_pair(obj, parts)

    # pass 'A1' to 'rhand'
    _,_,handwidth = graspplan(parts) # from remote
    pass_left_to_right(parts, handwidth)

    # put 'A1' to 'P0'
    jts = 'torso_rarm'
    place = env.get_object(name=name2)
    afrm,gfrm = placeplan(place, parts)
    if not move_arm2(afrm, gfrm, joints=jts, width=handwidth+20):
        afrm,gfrm = request_next(afrm, gfrm)
        if not move_arm2(afrm, gfrm, joints=jts, width=handwidth+20):
            return False
        
    unfix(parts, hand=second)
    move_arm(afrm, joints=jts, check_collision=False, duration=0.2)
    r.release_collision_object(parts, hand=second)
    for obj in env.get_objects('table top|pallete side|A|B'):
        if obj.name != parts.name:
            r.remove_collision_pair(obj, parts)

    return True

def reset_parts():
    def reset1(nm, pose):
        o = env.get_object(nm)
        o.unfix()
        o.affix(env.get_object('table'), FRAME(xyzabc=pose))

    for i,pose in enumerate([[-260,-50,714,0,0,pi/6],
                             [-170,100,714,0,0,-pi/6],
                             [-250,190,714,0,0,0],
                             [-120,-10,714,0,0,pi/4]]):
        reset1('A'+str(i), pose)
    for i,pose in enumerate([[-160,210,700,0,0,0],
                             [-120,-70,700,0,0,0]]):
        reset1('B'+str(i), pose)
        

colored_print("approach('A0')", 'blue')
colored_print("approach('P0')", 'blue')
colored_print("approach('B0', joints='torso_larm')", 'blue')
colored_print("plan('A2','P3')", 'blue')
colored_print("plan('B1','P1', joints='torso_larm')", 'blue')
colored_print("pick_and_place('A0', 'P0', joints='rarm')", 'blue')
colored_print("go_prepare_pose()", 'blue')
colored_print("pick_pass_and_place('A1', 'P2')", 'blue')
colored_print("reset_parts()", 'blue')
colored_print("palletize()", 'blue')
colored_print("reset_parts()", 'blue')
colored_print("palletize(task_sequence=[('B0','P0','left'),('A2','P3','right')])", 'blue')
