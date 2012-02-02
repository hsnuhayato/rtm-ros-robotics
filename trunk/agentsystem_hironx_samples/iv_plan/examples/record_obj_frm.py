# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

def detect_AR_tags_with_kinect():
    res = rr.detect(camera='kinect_AR')

    frms = []
    r.set_joint_angles(rr.get_joint_angles())
    for objnum,Tcam_mk in res:
        Tcam_box = Tcam_mk
        print 'kinect->target:', Tcam_box
        Twld_box = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectrgb*Tcam_box
        print 'world->target:', Twld_box
        frms.append((objnum,Twld_box))
        # 認識位置の可視化
        name = 'box'+str(objnum)
        env.delete_object(name)
        bx = visual.box(length=97, height=66, width=57, color=(1,0,1))
        obj = PartsObjectWithName(vbody=bx,name=name)
        env.insert_object(obj, Twld_box, env.get_world())

    return frms

def detect_center_with_Kinect():
    if rr:
        Tkinect_cb = rr.detect(camera='kinect_point_center')
        r.set_joint_angles(rr.get_joint_angles())
        print 'kinect_point_center->target:', Tkinect_cb
        Twld_cb = r.get_link('HEAD_JOINT1_Link').where()*r.Thd_kinectdepth*Tkinect_cb[0]
        Twld_cb.mat = MATRIX()
        print 'world->target:', Twld_cb
        show_frame(Twld_cb, 'center')
        return Twld_cb
    else:
        warn('not yet supported')

def record_obj_frm(mode='AR_tags', filename='/home/leus/workspace/grasp_set/obj_frm.txt', use_two_hands=False):
    if (use_two_hands==True):
        f = open('/home/leus/workspace/grasp_set_2/obj_frm.txt', 'w')
    else:
        f = open(filename, 'w')

    if mode == 'AR_tags':
        obj_frm = detect_AR_tags_with_kinect()[0][1]
    if mode == 'center_point':
        obj_frm = detect_center_with_Kinect()

    m = obj_frm.mat
    v = obj_frm.vec
    for s in m:
        for x in s[:-1]:
            f.write(str(x))
            f.write(' ')
        f.write(str(s[len(s)-1]))
        f.write('\n')
    for s in v[:-1]:
        f.write(str(s))
        f.write(' ')
    f.write(str(v[len(v)-1]))
    f.write('\n')
    f.close()

