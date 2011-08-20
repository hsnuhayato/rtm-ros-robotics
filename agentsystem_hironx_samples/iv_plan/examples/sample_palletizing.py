# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def approach(partsname):
    '''make a plan to a parts or a place from prepare position'''
    prepare() # initial pose
    r.set_joint_angle(0, 0.8) # turn the waist
    p1 = env.get_object(partsname).where() # get a goal frame of the end-effector
    p1.vec[2] += 30 # approach frame
    p1 = p1 * (-r.Twrist_ef) # calc a corresponding wrist frame
    q0 = r.get_arm_joint_angles()
    q1 = r.ik(p1)[0]
    traj = pl.make_plan(q0, q1)
    if traj:
        show_traj(traj[1])
        return traj

def plan(name1='A0', name2='P0'):
    def get_config(name):
        p = env.get_object(name).where()
        p.vec[2] += 20
        return r.ik(p*(-r.Twrist_ef))[0]

    r.set_joint_angle(0, 0.8)
    traj = pl.make_plan(get_config(name1), get_config(name2))
    if traj:
        show_traj(traj[1])
        return traj

def graspplan(parts):
    gfrm = parts.where()*(-r.Twrist_ef)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    handwidth = parts.vbody.size[1]
    return afrm,gfrm,handwidth

def placeplan(place):
    gfrm = place.where()*(-r.Twrist_ef)
    gfrm.vec[2] += 20
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    return afrm,gfrm

def demo(n=2):
    # parts: A0,A1,A2,A3, places: P0,P1,P2,P3
    # task sequence:
    #  A0 => P0, A1 => P1, A2 => P2, A3 => P3
    
    prepare()
    r.set_joint_angle(0, 0.8)
    p0 = r.fk()

    for i in range(n):
        l_or_r = 'right'
        parts = env.get_object(name='A'+str(i))
        afrm,gfrm,handwidth = graspplan(parts)
        move_arm(afrm, arm=l_or_r, check_collision=True)
        move_arm(gfrm, arm=l_or_r, check_collision=False)
        grasp(width=handwidth, name=parts.name, hand=l_or_r)
        move_arm(gfrm, arm=l_or_r, check_collision=False)

        # for collision check ( get objects using regular expression )
        remove_hand_collision(parts.name)
        for obj in env.get_objects('table top|pallete side|A'):
            if obj.name != parts.name:
                r.add_collision_pair(obj, parts)
        
        place = env.get_object(name='P'+str(i))
        afrm,gfrm = placeplan(place)
        move_arm(afrm, arm=l_or_r, check_collision=True)
        move_arm(gfrm, arm=l_or_r, check_collision=False)
        release(width=parts.vbody.size[1]+10, name=parts.name, hand=l_or_r)
        move_arm(gfrm, arm=l_or_r, check_collision=False)        

        # for collision check
        add_hand_collision(parts.name)
        for obj in env.get_objects('table top|pallete side|A'):
            if obj.name != parts.name:
                r.remove_collision_pair(obj, parts)

    move_arm(p0, arm=l_or_r, check_collision=True)

def pick_with_handcam():
    prepare()
    sync()
    f = FRAME(xyzabc=[250,-50,850,0,0,0])
    move_arm_ef(f)
    Trhandcam_tgt = FRAME(xyzabc=[0,0,220,pi,0,pi/6]) # replaced with recognition result
    Twld_tgt = r.get_sensor('rhandcam').where()*Trhandcam_tgt
    move_arm_ef(Twld_tgt, duration=4.0)

def reset_world():
    for i in range(4):
        env.delete_object('A'+str(i))
    env.delete_object('table')
    env.delete_object('floor')
    env.delete_object('hirobase')
    env.load_scene(scene_objects.ac_scene())


colored_print("approach('A0')", 'blue')
colored_print("approach('A1')", 'blue')
colored_print("approach('A2')", 'blue')
colored_print("approach('A3')", 'blue')
colored_print("approach('P0')", 'blue')
colored_print("approach('P1')", 'blue')
colored_print("approach('P2')", 'blue')
colored_print("approach('P3')", 'blue')
colored_print("plan('A0','P0')", 'blue')
colored_print('demo()', 'blue')
