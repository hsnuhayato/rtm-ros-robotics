#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import random
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


def putbox(name='box0', vaxis='x', pose2d=None):
    '''シミュレータ内で箱を机上に置く。位置はランダムに決定される。vaxis="y"で側面を上に向けて置く'''
    if pose2d:
        x,y,theta  = pose2d
    else:
        x = random.uniform(-300,-50)
        y = random.uniform(-200,200)
        theta = random.uniform(0,2*pi)
    env.delete_object(name)
    bl,bh,bw=97,66,57
    bx = visual.box(length=bl, height=bh, width=bw, color=(1,0,1))
    obj = PartsObjectWithName(vbody=bx, name=name)
    tbltop = env.get_object('table top') # テーブル上面
    thickness = tbltop.vbody.size[2]
    if vaxis == 'x':
        relfrm = FRAME(xyzabc=[x,y,(thickness+bw)/2,0,0,theta])
    elif vaxis == 'y':
        relfrm = FRAME(xyzabc=[x,y,(thickness+bh)/2,pi/2,theta,0])
    elif vaxis == 'z':
        relfrm = FRAME(xyzabc=[x,y,(thickness+bh)/2,0,-pi/2,0])*FRAME(xyzabc=[0,0,0,theta,0,0])
    else:
        print 'vaxis is wrong'
    return env.insert_object(obj, relfrm, tbltop)

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

def release(hand='right', width=80, unfixobj=True,name='box0'):
    r.grasp(width=width, hand=hand)
    if unfixobj:
        tgtobj = env.get_object(name)
        worldfrm = tgtobj.where()
        tgtobj.unfix()
        tgtobj.affix(env.get_world(), worldfrm)

def grasp(hand='right', width=62, affixobj=True, name='box0'):
    r.grasp(width=width, hand=hand)
    if affixobj:
        tgtobj = env.get_object(name)
        if hand=='right':
            handjnt = r.get_joint('RARM_JOINT5')
        else:
            handjnt = r.get_joint('LARM_JOINT5')
        reltf = (-handjnt.where())*tgtobj.where()
        tgtobj.unfix()
        tgtobj.affix(handjnt, reltf)

def move_arm_plan(p1):
    '''move arm from current pose to p1'''
    q0 = r.get_arm_joint_angles()
    q1 = r.ik(p1)[0]
    traj = pl.make_plan(q0, q1)
    if traj:
        show_traj(traj[1])
        return traj

def move_arm(f, duration=2.0, arm='right', width=None, use_waist=True, check_collision=False):
    if check_collision:
        traj = move_arm_plan(f)
        exec_traj(traj[1], duration=0.1)
        return
    
    if use_waist:
        w,avec = r.ik(f,arm=arm,use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(avec,arm=arm)
    else:
        r.set_arm_joint_angles(r.ik(f,arm=arm)[0], arm=arm)

    if width:
        r.grasp(width=width, hand=arm)
        if arm == 'right':
            hand_joints = '_rhand'
        else:
            hand_joints = '_lhand'
    else:
        hand_joints = ''
    if arm == 'right':
        arm_joints = 'rarm'
    else:
        arm_joints = 'larm'
    if use_waist:
        torso_joints = 'torso_'
    else:
        torso_joints = ''

    print torso_joints+arm_joints+hand_joints
    sync(duration=duration, joints=torso_joints+arm_joints+hand_joints, waitkey=False)

def move_arm_ef(f, duration=2.0, arm='right', width=None, use_waist=True, check_collision=False):
    f = f*(-r.Twrist_ef)
    move_arm(f, duration=duration, arm=arm, width=width,
             use_waist=use_waist, check_collision=check_collision)

def set_view(camera='world'):
    warn('not yet implemented')
    return
    if camera == 'world':
        pass
    elif camera == 'leye':
        pass

def show_frame(frm, name='frame0'):
    '''フレームを可視化する。名前を指定しないときは"frame0"という名前のオブジェクトを作り環境に挿入する'''
    env.delete_object(name)
    bx = visual.box(length=10, height=10, width=10, color=(1,0,1))
    obj = PartsObjectWithName(vbody=bx,name=name)
    obj.vframe.resize(60.0)
    env.insert_object(obj, frm, env.get_world())

##
##

def show_traj(sts, name='traj0'):
    env.delete_object(name)
    traj = CoordinateObjects(name)
    for st in sts:
        r.set_arm_joint_angles(st.avec)
        f = r.fk()
        traj.append(f)
    env.insert_object(traj, FRAME(), env.get_world())

def show_tree():
    show_traj(pl.T_init, name='traj0')
    show_traj(pl.T_goal, name='traj1')

def exec_traj(traj, duration=0.8, use_armcontrol=False):
    def robot_relative_traj(traj):
        T = -r.get_link('WAIST_Link').where()
        qs = [x.avec for x in traj]
        ps = []
        for q in qs:
            r.set_arm_joint_angles(q)
            ps.append(T*r.get_link('RARM_JOINT5_Link').where())
        return ps

    if use_armcontrol:
        rr.send_trajectory(robot_relative_traj(traj), duration=duration)
    else:
        avecs = [x.avec for x in traj]
        for avec in avecs:
            r.set_arm_joint_angles(avec)
            sync(duration=duration, waitkey=False)

def setup_collision_objects():
    # table top <=> robot
    # pallete side <=> robot
    # parts <=> robot
    for obj in env.get_objects('table top|pallete side|A'):
        r.add_collision_object(obj)

def add_hand_collision(objname):
    obj = env.get_object(objname)
    for lnknm in ['RHAND_JOINT0_Link', 'RHAND_JOINT1_Link',
                  'RHAND_JOINT2_Link', 'RHAND_JOINT3_Link']:
        lnk = r.get_link(lnknm)
        r.add_collision_pair(lnk, obj)

def remove_hand_collision(objname):
    obj = env.get_object(objname)
    for lnknm in ['RHAND_JOINT0_Link', 'RHAND_JOINT1_Link',
                  'RHAND_JOINT2_Link', 'RHAND_JOINT3_Link']:
        lnk = r.get_link(lnknm)
        r.remove_collision_pair(lnk, obj)

def add_objects_collision(objname1, objname2):
    obj1 = env.get_object(objname1)
    obj2 = env.get_object(objname2)
    r.add_collision_pair(obj1, obj2)

def remove_objects_collision(objname1, objname2):
    obj1 = env.get_object(objname1)
    obj2 = env.get_object(objname2)
    r.remove_collision_pair(obj1, obj2)

def prepare():
    r.set_joint_angles(r.poses['prepare'])

def prepare_right():
    r.set_joint_angles(r.poses['prepare_right'])


setup_collision_objects()
