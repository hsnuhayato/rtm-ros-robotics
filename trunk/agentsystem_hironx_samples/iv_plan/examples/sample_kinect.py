# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *


def detect_kinect_rgb():
    '''チェスボードが貼られた箱の認識'''
    if rr:
        Tkinect_cb = rr.detect(camera='kinect_rgb')
        r.set_joint_angles(rr.get_joint_angles())
        print 'kinect_rgb->target:', Tkinect_cb
        Twld_cb = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectrgb*Tkinect_cb
        print 'world->target:', Twld_cb

        show_frame(Twld_cb)
        return Twld_cb
    else:
        warn('not yet supported')

def detect_kinect_point_center():
    if rr:
        Tkinect_cb = rr.detect(camera='kinect_point_center')
        r.set_joint_angles(rr.get_joint_angles())
        print 'kinect_point_center->target:', Tkinect_cb
        Twld_cb = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*Tkinect_cb
        print 'world->target:', Twld_cb

        show_frame(Twld_cb)
        return Twld_cb
    else:
        warn('not yet supported')


#r.set_joint_angles(prepare_right2)
prepare_right2 = [0.24999878039410275,
                  -0.3999701506333021,
                  1.0999912242536529,
                  -0.242129102211388,
                  -0.55000849205344104,
                  -2.246649011302321,
                  -0.23170880440775701,
                  0.77617486969986105,
                  -0.064163644362093999,
                  0.009992009967667536,
                  0.0,
                  -1.7449976394364506,
                  -0.26498055477880189,
                  0.16399277276356095,
                  -0.055982450484027418,
                  0.78539814154175325,
                  -0.089011788368225098,
                  -0.78539814154175325,
                  0.090757124125957489,
                  -4.3711390063094768e-08,
                  0.013962633907794952,
                  0.0069812973353524654,
                  -0.015707964077591896]


def grasp_pet_with_kinect(f=None):
    if rr:
        f = detect_kinect_point_center()

    if f:
        fa = FRAME(xyzabc=[f.vec[0]-130,f.vec[1]-100,850,0,pi+0.1,-0.5])
        fg = FRAME(xyzabc=[f.vec[0]-55,f.vec[1]-50,850,0,pi+0.1,-0.5])
        w,av = r.ik(fa, use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(av)
        r.grasp(width=100)
        sync()
        w,av = r.ik(fg, use_waist=True)[0]
        r.set_joint_angle(0, w)
        r.set_arm_joint_angles(av)
        sync(duration=2.0)
        r.grasp(width=58)
        sync(duration=2.0)
        fp = r.fk()
        fp.vec[2] += 50
        r.set_arm_joint_angles(r.ik(fp)[0])
        sync(joints='rarm', duration=2.0)

def grasp_yellow_piece_with_kinect(f=None):
    if rr:
        f = detect_kinect_point_center()

    if f:
        fa = FRAME(xyzabc=[f.vec[0],f.vec[1],950,0,-pi/2,0])
        fg = FRAME(xyzabc=[f.vec[0],f.vec[1],900,0,-pi/2,0])
        r.set_arm_joint_angles(r.ik(fa)[0])
        r.grasp(width=100)
        sync()
        r.set_arm_joint_angles(r.ik(fg)[0])
        sync(duration=2.0)
        r.grasp(width=24)
        sync(duration=2.0)
        fp = r.fk()
        fp.vec[2] += 50
        r.set_arm_joint_angles(r.ik(fp)[0])
        sync(joints='rarm', duration=2.0)

