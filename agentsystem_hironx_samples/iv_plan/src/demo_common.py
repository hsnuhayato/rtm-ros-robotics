#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import re

import set_env
from ivutils import *
from viewer import *
import scene_objects
from robot import *
from mplan_env import *
from csplan import *


if real_robot:
    from real_hiro import *
    rr = RealHIRO(set_env.nameserver)
else:
    rr = None

env = MPlanEnv()
env.load_scene(scene_objects.table_scene())
r = VHIRONX(ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/')
env.insert_robot(r)
r.go_pos(-150, 0, 0)
pl = CSPlanner(env)


def sync_main(duration=4.0, joints='all', wait=True, waitkey=True):
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
        elif joints == 'rarm_rhand':
            rr.send_goal([[],js[3:9],[],js[15:19],[]], duration, wait=wait)
        elif joints == 'torso_rarm_rhand':
            rr.send_goal([js[0:3],js[3:9],[],js[15:19],[]], duration, wait=wait)
        elif joints == 'larm_lhand':
            rr.send_goal([[],[],js[9:15],[],js[19:23]], duration, wait=wait)
        elif joints == 'torso_larm_lhand':
            rr.send_goal([js[0:3],[],js[9:15],[],js[19:23]], duration, wait=wait)
        elif joints == 'torso_arms':
            rr.send_goal([js[0:3],js[3:9],js[9:15],[],[]], duration, wait=wait)
        elif joints=='all':
            rr.send_goal([js[0:3],js[3:9],js[9:15],js[15:19],js[19:23]], duration, wait=wait)
        else:
            warn('unknown joints parameter: ' + joints)
    else:
        if waitkey:
            raw_input('type any key to continue')
        else:
            time.sleep(duration)

# def sync(duration=4.0, joints='all', wait=True, waitkey=True, goalthresh=0.2):
#     while True:
#         sync_main(duration=duration, joints=joints, wait=wait, waitkey=waitkey)
#         qs = r.get_joint_angles(joints='all')
#         qr = rr.get_joint_angles()
#         if numpy.linalg.norm(array(qr) - array(qs)) < goalthresh:
#             return
#         else:
#             time.sleep(1)

def sync(duration=4.0, joints='all', wait=True, waitkey=True, goalthresh=0.2):
    sync_main(duration=duration, joints=joints, wait=wait, waitkey=waitkey)


# def release(hand='right', width=80, unfixobj=True,name='box0'):
#     r.grasp(width=width, hand=hand)
#     if unfixobj:
#         tgtobj = env.get_object(name)
#         affix(tgtobj, hand=hand)

# def grasp(hand='right', width=62, affixobj=True, name='box0'):
#     r.grasp(width=width, hand=hand)
#     if affixobj:
#         tgtobj = env.get_object(name)
#         affix(tgtobj, hand=hand)


def affix(obj, hand='right'):
    if hand == 'right':
        handjnt = r.get_joint('RARM_JOINT5')
    else:
        handjnt = r.get_joint('LARM_JOINT5')
    reltf = (-handjnt.where())*obj.where()
    obj.parent.children.remove(obj)
    obj.unfix()
    obj.affix(handjnt, reltf)


def unfix(obj, hand='right'):
    wldfrm = obj.where()
    obj.unfix()
    wld = env.get_world()
    wld.children.append(obj)
    obj.affix(wld, wldfrm)


def move_arm_plan(p1, joints='rarm'):
    '''move arm from current pose to p1'''
    q0 = r.get_joint_angles(joints=joints)
    q1 = r.ik(p1, joints=joints)[0]
    return pl.make_plan(q0, q1, joints=joints)


def move_arm(f, duration=2.0, joints='rarm', width=None, check_collision=False):
    if check_collision:
        traj = move_arm_plan(f, joints=joints)
        exec_traj(traj, joints=joints, duration=0.05)
    else:
        q = r.ik(f, joints=joints)[0]
        r.set_joint_angles(q, joints=joints)
        sync(duration=duration, joints=joints, waitkey=False)

    if width:
        rl,use_waist = parse_joints_flag(joints)
        r.grasp(width=width, hand=rl)
        sync(duration=0.5, joints=rl[0]+'hand', waitkey=False)

    return True


def move_arm2(afrm, gfrm, width, duration=2.0, joints='torso_rarm'):
    if r.ik(afrm, joints) == [] or r.ik(gfrm, joints) == []:
        return False
    else:
        move_arm(afrm, joints=joints, check_collision=True, duration=duration)
        move_arm(gfrm, width=width, joints=joints, check_collision=False, duration=0.5)
        return True


def go_prepare_pose():
    jts = 'all'
    q0 = r.get_joint_angles(joints=jts)
    r.prepare()
    q1 = r.get_joint_angles(joints=jts)
    traj = pl.make_plan(q0, q1, joints=jts)
    if traj:
        exec_traj(traj, joints=jts)
        return True
    else:
        warn('error: go_prepare_pose()')
        return False


def show_frame(frm, name='frame0'):
    env.delete_object(name)
    bx = visual.box(length=10, height=10, width=10, color=(1,0,1))
    obj = PartsObjectWithName(vbody=bx,name=name)
    obj.vframe.resize(60.0)
    env.insert_object(obj, frm, env.get_world())


def show_traj(sts, joints='rarm', name='traj0'):
    env.delete_object(name)
    traj = CoordinateObjects(name)
    for st in sts:
        r.set_joint_angles(st.avec, joints=joints)
        if re.match('.*rarm$', joints) or joints == 'all':
            f = r.fk('right')
            traj.append(f)
        if re.match('.*larm$', joints) or joints == 'all':
            f = r.fk('left')
            traj.append(f)
    env.insert_object(traj, FRAME(), env.get_world())


def show_tree():
    show_traj(pl.T_init, name='traj0')
    show_traj(pl.T_goal, name='traj1')


def exec_traj(traj, duration=0.05, joints='rarm', use_armcontrol=False, draw_trajectory=True):
    def robot_relative_traj(traj):
        T = -r.get_link('WAIST_Link').where()
        qs = [x.avec for x in traj]
        ps = []
        for q in qs:
            r.set_joint_angles(q, joints=joints)
            ps.append(T*r.get_link('RARM_JOINT5_Link').where())
        return ps

    name = 'last_trajectory'
    env.delete_object(name)
    frames = CoordinateObjects(name)

    if rr:
        duration = 0.15

    if use_armcontrol:
        rr.send_trajectory(robot_relative_traj(traj), duration=duration)
    else:
        for st in traj:
            r.set_joint_angles(st.avec, joints=joints)
            sync(duration=duration, joints=joints, waitkey=False)

            if draw_trajectory:
                if re.match('.*rarm$', joints) or joints == 'torso_arms' or joints == 'all':
                    f = r.fk('right')
                    frames.append(f)
                if re.match('.*larm$', joints) or joints == 'torso_arms' or joints == 'all':
                    f = r.fk('left')
                    frames.append(f)
        env.insert_object(frames, FRAME(), env.get_world())


def objtype(obj):
    if re.match('^A.*', obj.name):
        return 1
    elif re.match('^B.*', obj.name):
        return 2
    elif re.match('^P.*', obj.name):
        return 3
    else:
        return 0


def obj_in_hand(hand='right'):
    prefix = 'R' if hand == 'right' else 'L'
    handlinks = [r.get_link('%sHAND_JOINT%d_Link'%(prefix, n)) for n in [1,3]]
    for obj in env.collidable_objects:
        for l in handlinks:
            if in_collision_pair_parts(l, obj, {}):
                warn('grab %s'%obj)
                return obj
    warn('failed to grab')
    return None


def grab(hand='right'):
    obj = obj_in_hand(hand=hand)
    if obj == None:
        return False
    r.grabbed_obj[hand] = obj
    affix(obj, hand=hand)
    r.grasp_collision_object(obj, hand=hand)
    for obj2 in env.collidable_objects:
        if obj.name != obj2.name:
            r.add_collision_pair(obj2, obj)
    return True


def release(hand='right'):
    obj = r.grabbed_obj[hand]
    if obj == None:
        return False
    warn('release %s'%obj)
    r.grabbed_obj[hand] = None
    unfix(obj, hand=hand)
    r.release_collision_object(obj, hand=hand)
    # for obj2 in env.collidable_objects:
    #     if obj.name != obj2.name:
    #         r.remove_collision_pair(obj2, obj)
    return True

