#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import re
from ivplan import *

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
        elif joints == 'rarm_rhand':
            rr.send_goal([[],js[3:9],[],js[15:19],[]], duration, wait=wait)
        elif joints == 'torso_rarm_rhand':
            rr.send_goal([js[0:3],js[3:9],[],js[15:19],[]], duration, wait=wait)
        elif joints == 'larm_lhand':
            rr.send_goal([[],[],js[9:15],[],js[19:23]], duration, wait=wait)
        elif joints == 'torso_larm_lhand':
            rr.send_goal([js[0:3],[],js[9:15],[],js[19:23]], duration, wait=wait)
        elif joints=='all':
            rr.send_goal([js[0:3],js[3:9],js[9:15],js[15:19],js[19:23]], duration, wait=wait)
        else:
            warn('unknown joints parameter: ' + joints)
    else:
        if waitkey:
            raw_input('type any key to continue')
        else:
            time.sleep(duration)


def detect(name='box0'):
    if rr:
        Tleye_cb = rr.detect(camera='leye')
        bl,bh,bw = 97,66,57
        Tcb_box = FRAME(xyzabc=[12,-8,-bw/2.0,0,0,pi])
        Tleye_box = Tleye_cb*Tcb_box
        r.set_joint_angles(rr.get_joint_angles())
        print 'leye->target:', Tleye_box
        Twld_box = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_leye*Tleye_box
        print 'world->target:', Twld_box

        # 認識位置の可視化
        env.delete_object(name)
        bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
        obj = PartsObjectWithName(vbody=bx,name=name)
        env.insert_object(obj, Twld_box, env.get_world())

        return Twld_box
    else:
        obj = env.get_object(name)
        if obj:
            frm2 = obj.where()
            print "world->target:", frm2
            return frm2
        else:
            print "not detected"
            return None


def detect_rhand():
    '''ARマーカが貼られた箱の認識(複数対応)'''
    if rr:
        res = rr.detect(camera='rhand')
        bl,bh,bw = 97,66,57
        Tmk_box = FRAME(xyzabc=[0,0,-bh/2.0,pi/2,0,0])
        frms = []
        r.set_joint_angles(rr.get_joint_angles())
        for objnum,Tcam_mk in res:
            Tcam_box = Tcam_mk*Tmk_box
            print 'rhand->target:', Tcam_box
            Twld_box = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam*Tcam_box
            print 'world->target:', Twld_box
            frms.append((objnum,Twld_box))
            # 認識位置の可視化
            name = 'box'+str(objnum)
            env.delete_object(name)
            bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
            obj = PartsObjectWithName(vbody=bx,name=name)
            env.insert_object(obj, Twld_box, env.get_world())

        return frms
    else:
        # box*という名前の物体を検出する
        # *の部分がマーカ番号
        def detected(obj):
            x,y,z = obj.where().vec
            return z > 700 and re.match('box*', obj.name)

        detected_objs = [x for x in env.get_objects() if detected(x)]
        return [(int(re.sub('box', '', x.name)), x.where()) for x in detected_objs]


def detect_rhand2():
    '''ARマーカが貼られた箱の認識(複数対応)'''
    if rr:
        res = rr.detect(camera='rhand')
        bl,bh,bw = 97,66,57
        Tmk_box = FRAME(xyzabc=[0,0,-bw/2.0,0,0,0])
        frms = []
        r.set_joint_angles(rr.get_joint_angles())
        for objnum,Tcam_mk in res:
            Tcam_box = Tcam_mk*Tmk_box
            print 'rhand->target:', Tcam_box
            Twld_box = r.get_link('RARM_JOINT5_Link').where()*r.Trh_cam*Tcam_box
            print 'world->target:', Twld_box
            frms.append((objnum,Twld_box))
            # 認識位置の可視化
            name = 'box'+str(objnum)
            env.delete_object(name)
            bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
            obj = PartsObjectWithName(vbody=bx,name=name)
            env.insert_object(obj, Twld_box, env.get_world())

        return frms
    else:
        # box*という名前の物体を検出する
        # *の部分がマーカ番号
        def detected(obj):
            x,y,z = obj.where().vec
            return z > 700 and re.match('box*', obj.name)

        detected_objs = [x for x in env.get_objects() if detected(x)]
        return [(int(re.sub('box', '', x.name)), x.where()) for x in detected_objs]


def look_for_boxes(name='box0'):
    '''右手ハンドの向きを変えて箱（マーカ）を探す'''
    f0 = r.fk()
    objfrms = [None,None]
    for i in range(1,2)+range(2,-4,-1):
        f = f0 * FRAME(xyzabc=[0,0,0,0,0,pi/16*i])
        js = r.ik(f)[0]
        r.set_arm_joint_angles(js)
        sync(duration=1.5)

        for objnum, objfrm in detect_rhand():
            print 'marker %d found'%objnum
            if objnum < 2:
                objfrms[objnum] = objfrm
                print objfrms
            if objfrms[0] and objfrms[1]:
                return objfrms
    return None


def look_for_boxes2(num):
    '''右手ハンドの向きを変えて箱（マーカ）を探す'''
    f0 = r.fk()
    objfrms = [None,None]
    for i in range(1,2)+range(2,-4,-1):
        f = f0 * FRAME(xyzabc=[0,0,0,0,0,pi/16*i])
        js = r.ik(f)[0]
        r.set_arm_joint_angles(js)
        sync(duration=1.5)

        for objnum, objfrm in detect_rhand2():
            print 'marker %d found'%objnum
            if objnum == num:
                print objfrm
                return objfrm
    return None


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
    obj.unfix()
    obj.affix(handjnt, reltf)


def unfix(obj, hand='right'):
    wldfrm = obj.where()
    obj.unfix()
    obj.affix(env.get_world(), wldfrm)


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
        duration = 0.2

    if use_armcontrol:
        rr.send_trajectory(robot_relative_traj(traj), duration=duration)
    else:
        for st in traj:
            r.set_joint_angles(st.avec, joints=joints)
            sync(duration=duration, waitkey=False)

            if draw_trajectory:
                if re.match('.*rarm$', joints) or joints == 'all':
                    f = r.fk('right')
                    frames.append(f)
                if re.match('.*larm$', joints) or joints == 'all':
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
    for obj in env.get_objects('A|B'):
        for l in handlinks:
            if in_collision_pair(l, obj, {}):
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
    for obj2 in env.get_objects('table top|pallete side|A|B'):
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
    for obj2 in env.get_objects('table top|pallete side|A|B'):
        if obj.name != obj2.name:
            r.remove_collision_pair(obj2, obj)
    return True


def graspplan(objtype, objfrm, long_side=False):
    if objtype == 1:
        if long_side:
            objfrm = objfrm * FRAME(xyzabc=[0,0,0,0,0,pi/2])
            handwidth = 48
        else:
            handwidth = 38
        gfrm = objfrm*(-r.Twrist_ef)
        afrm = FRAME(gfrm)
        afrm.vec[2] += 40
    else:
        handwidth = 25
        objfrm = objfrm * FRAME(vec=[0,0,58-15])
        gfrm = objfrm*(-r.Twrist_ef)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40

    return afrm,gfrm,handwidth


def request_next(afrm, gfrm, handwidth):
    return afrm*FRAME(xyzabc=[0,0,0,pi,0,0]), gfrm*FRAME(xyzabc=[0,0,0,pi,0,0]), handwidth


def placeplan(objtype, plcfrm):
    # plcfrm = place.where()
    gfrm = plcfrm*(-r.Twrist_ef)
    if objtype == 1:
        gfrm.vec[2] += 28/2
    else:
        gfrm.vec[2] += (58-15)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    return afrm,gfrm


def reset_parts(env):
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


def setup_collision_objects():
    # table top <=> robot
    # pallete side <=> robot
    # parts <=> robot
    for obj in env.get_objects('table top|pallete side|A|B'):
        r.add_collision_object(obj)

setup_collision_objects()
