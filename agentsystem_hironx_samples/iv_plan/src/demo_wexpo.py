# -*- coding: utf-8 -*-

from numpy import *
import sys
import time

from demo_common import *
from setup_rtchandle import *


rhandhost = 'VisionPC'
lhandhost = 'lupus'

def init_palletizing_scene():
    tbl = env.get_object('table')

    def put_on_table(objdef, name, xyzabc):
        obj = env.eval_sctree(objdef(name=name))
        env.insert_object(obj, FRAME(xyzabc=xyzabc), tbl)

    # put_on_table(scene_objects.pallete, 'pallete0', [-220,-310,700,0,0,-pi/3])
    put_on_table(scene_objects.pallete, 'pallete0', [-290,-270,700,0,0,0])

    put_on_table(scene_objects.partsA, 'A0', [-330,-10,716,0,0,pi/6])
    put_on_table(scene_objects.partsA, 'A1', [-280,100,716,0,0,-pi/6])
    put_on_table(scene_objects.partsA, 'A2', [-310,190,716,0,0,0])
    put_on_table(scene_objects.partsA, 'A3', [-250,-10,716,0,0,pi/4])

    # put_on_table(scene_objects.partsB, 'B0', [-160,210,700,0,0,0])
    # put_on_table(scene_objects.partsB, 'B1', [-120,-70,700,0,0,0])

    plt  = env.get_object('pallete0')

    def put_on_pallete(objdef, name, xyzabc):
        obj = env.eval_sctree(objdef(name=name))
        env.insert_object(obj, FRAME(xyzabc=xyzabc), plt)

    put_on_pallete(scene_objects.rect_pocket, 'P0', [40,40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P1', [-40,40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P2', [40,-40,20,0,0,0])
    put_on_pallete(scene_objects.rect_pocket, 'P3', [-40,-40,20,0,0,0])

    for o in env.get_objects('table top|pallete side|A|B'):
        r.add_collision_object(o)

init_palletizing_scene()


try:
    hrhandrecog = ns.rtc_handles[rhandhost+'.host_cxt/AppRecog0.rtc'].outports['AppRecog0.RecognitionResultOut']
except:
    warn('recognition module is not running in rhand')

try:
    hlhandrecog = ns.rtc_handles[lhandhost+'.host_cxt/AppRecog0.rtc'].outports['AppRecog0.RecognitionResultOut']
except:
    warn('recognition module is not running in lhand')


# quick version
# tms = {'preapproach1': 0.4,
#        'preapproach2': 1.0,
#        'pick': 0.65,
#        'transport': 0.6,
#        'place': 0.55,
#        'look_for': 0.5}

# slow version
tms = {'preapproach1': 1.5,
       'preapproach2': 2.5,
       'pick': 1.3,
       'transport': 1.6,
       'place': 1.3,
       'pregrasp': 0.7,
       'look_for': 0.8}

detectposs = [(190,-60),(260,-60),
              (180, 20),(250, 20)]


detectposs_dual = [[(180,-35),(180,155)],
                   [(260,-35),(260,155)]]

# pocketposs = [(200,-300),(120,-300),
#               (200,-380),(120,-380)]
pocketposs = [(160,-240),(80,-240),
              (160,-330),(80,-330)]

tblheight = 700
fsoffset = 59

def preapproach(n = 0, height=tblheight+fsoffset+290):
    print 'PRE:', n
    f = r.fk()
    f.vec[2] += 60
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=tms['preapproach1'])

    r.prepare(width=80)
    x,y = detectposs[n]
    f = FRAME(xyzabc=[x, y, height, 0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync(duration=tms['preapproach2'])

def detect_pose3d(scl=1.0, hand='right'):
    def read_stable_result(hrecog):
        lastpos = zeros(3)
        lasttm = RTC.Time(sec=0, nsec=0)
        while True:
            pose3d_stamped = hrecog.read()
            tm = pose3d_stamped.tm
            pose3d = pose3d_stamped.data

            if tm.sec > lasttm.sec or tm.nsec > lasttm.nsec:
                pos = array([pose3d.position.x, pose3d.position.y, pose3d.position.z])
                if linalg.norm(pos-lastpos) < 10:
                    break
                elif linalg.norm(pos) > 1:
                    lastpos = pos
                    lasttm = tm
                    time.sleep(0.1)

        return [pose3d.position.x, pose3d.position.y, pose3d.position.z,
                pose3d.orientation.r, pose3d.orientation.p, pose3d.orientation.y]

    if hand == 'right':
        hrecog = hrhandrecog
        parentlink = 'RARM_JOINT5_Link'
        Th_cam = r.Trh_cam
    else:
        hrecog = hlhandrecog
        parentlink = 'LARM_JOINT5_Link'
        Th_cam = r.Tlh_cam

    pose3d = read_stable_result(hrecog)
    if pose3d[2] < 100.0:
        return None

    x,y,z,ax,ay,az = pose3d
    axis =  array([ax,ay,az])
    angle = linalg.norm(axis)

    m = MATRIX(angle=angle, axis=VECTOR(vec=axis.tolist()))
    Tcam_obj = FRAME(mat=m, vec=VECTOR(x,y,z))

    print 'cam=>obj: ', Tcam_obj

    q = rr.get_joint_angles()
    r.set_joint_angles(q)
    Twld_cam = r.get_link(parentlink).where()*Th_cam

    Twld_obj = Twld_cam * Tcam_obj

    m = array(Twld_obj.mat)
    a = cross(m[0:3,2], [0,0,1])
    m2 = MATRIX(angle=linalg.norm(a), axis=VECTOR(vec=a.tolist()))
    Twld_obj.mat = m2*Twld_obj.mat

    return Twld_obj

def pick(obj, hand='right', height=tblheight+12):
    f = obj.where()
    if hand == 'right':
        Twrist_ef = r.Trwrist_ef
        jts = 'rarm'
        hjts = 'rhand'
    else:
        Twrist_ef = r.Tlwrist_ef
        jts = 'larm'
        hjts = 'lhand'

    f.vec[2] = height

    f2 = f * (-Twrist_ef)
    sols = r.ik(f2, joints=jts)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-Twrist_ef)
        sols = r.ik(f2, joints=jts)
    r.set_joint_angles(sols[0], joints=jts)
    sync(joints=jts, duration=tms['pick'])
    for w in [80,60,50,44,39,34]:
        r.grasp(w, hand=hand)
        sync(duration=0.2, joints=hjts)

def transport(n = 0):
    f = r.fk()
    f.vec[2] += 150
    sol = r.ik(f)[0]
    r.set_arm_joint_angles(sol)
    sync(joints='rarm', duration=tms['transport'])
    x,y = pocketposs[n]
    f = FRAME(xyzabc=[x, y, tblheight+300, 0,-pi/2,0])
    r.set_arm_joint_angles(r.ik(f)[0])
    sync(joints='rarm', duration=tms['transport'])

def place(f, h = tblheight+38+fsoffset, dosync=True):
    f.vec[2] = h
    f2 = f * (-r.Twrist_ef)
    sols = r.ik(f2)
    if sols == []:
        f2 = f * FRAME(xyzabc=[0,0,0,0,0,pi]) * (-r.Twrist_ef)
        sols = r.ik(f2)
    r.set_arm_joint_angles(sols[0])
    if dosync:
        sync(joints='rarm', duration=tms['place'])
        for w in [38,40,46,80]:
            r.grasp(w)
            sync(duration=0.2)

def detect(timeout=0, zmin=tblheight, zmax=tblheight+250,
           theta_constraint=None, hand='right'):
    start_tm = time.time()
    while timeout == 0 or time.time() - start_tm < timeout:
        f = detect_pose3d(hand=hand)
        if f and f.vec[2] > zmin and f.vec[2] < zmax:
            if theta_constraint != None:
                theta = acos(dot(array(f.mat)[0:2,0], array([1,0])))
                for c in theta_constraint:
                    if c[0] <= theta and theta <= c[1]:
                        return f
            else:
                return f

def look_for():
    r.prepare(width=80)
    detected = []
    for rpos,lpos in detectposs_dual:
        print rpos
        print lpos
        jts = 'rarm'
        fr = FRAME(xyzabc=[rpos[0], rpos[1], tblheight+fsoffset+290, 0, -pi/2,0])
        r.set_joint_angles(r.ik(fr, joints=jts)[0], joints=jts)
        jts = 'larm'
        fl = FRAME(xyzabc=[lpos[0], lpos[1], tblheight+fsoffset+290, 0, -pi/2,0])
        r.set_joint_angles(r.ik(fl, joints=jts)[0], joints=jts)
        sync(duration=tms['look_for'])
        obj_fr = detect(hand='right', timeout=1.5)
        obj_fl = detect(hand='left', timeout=1.5)
        if obj_fr:
            detected.append(obj_fr)
        if obj_fl:
            detected.append(obj_fl)

    for i,f in enumerate(detected):
        f.vec[2] = tblheight + 15
        env.get_object('A'+str(i)).locate(f, world=True)
    for i in range(len(detected),4): # not detected
        env.get_object('A'+str(i)).locate(FRAME(xyzabc=[500,-800,tblheight+16,0,0,0]))

    print '%d objects detected'%len(detected)
    if len(detected) < 4:
        return False
    else:
        return True

def choose_objs(n=0):
    def aux(o1, o2):
        x = o1.where()
        y = o2.where()
        return cmp(x.vec[0]-x.vec[1], y.vec[0]-y.vec[1])

    objs = [o for o in env.get_objects('^A') if o.where().vec[1] > -100]

    #if len(objs) == 4:
    if n == 0:
        objs.sort(cmp=aux)
        return objs[-1],objs[0]
    else:
        objs.sort(cmp=aux)
        o0 = objs[1]; o1 = objs[2]
        if o0.where().vec[1] < o1.where().vec[1]:
            return o0,o1
        else:
            return o1,o0

    return None

def grasp_plan(o, long_side=False):
    objfrm = o.where()

    if long_side:
        objfrm = objfrm * FRAME(xyzabc=[0,0,0,0,0,pi/2])
        gwidth = 45
    else:
        gwidth = 35

    gfrm = objfrm*(-r.Twrist_ef)
    afrm = FRAME(gfrm)
    afrm.vec[2] += 40
    awidth = 80.0

    afrm2 = afrm*FRAME(xyzabc=[0,0,0,pi,0,0])
    gfrm2 = gfrm*FRAME(xyzabc=[0,0,0,pi,0,0])
    return (afrm,gfrm,awidth,gwidth),(afrm2,gfrm2,awidth,gwidth)

def place_plan(p):
    plcfrm = p.where()

    gfrm = plcfrm*(-r.Twrist_ef)
    gfrm.vec[2] += 30/2

    afrm = FRAME(gfrm)
    afrm.vec[2] += 60
    rwidth = 80

    afrm2 = afrm*FRAME(xyzabc=[0,0,0,pi,0,0])
    gfrm2 = gfrm*FRAME(xyzabc=[0,0,0,pi,0,0])
    return (afrm,gfrm,rwidth),(afrm2,gfrm2,rwidth)

rwp = FRAME(xyzabc=[200,-110,1049,0,-pi/2,0])
lwp = FRAME(xyzabc=[240,90,1049,0,-pi/2,0])

def demo(recognition=False):
    preapproach_dual()

    if recognition:
        if not look_for():
            warn('detection failed')
            return

    o1, o2 = choose_objs(0)

    s1, s2 = grasp_plan(o1)
    jts = 'rarm'
    try:
        afrm,gfrm,awidth,gwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,awidth,gwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, right arm')
            return

    s1, s2 = grasp_plan(o2)
    jts = 'larm'
    try:
        afrm,gfrm,awidth,gwidth = s1
        lasol = r.ik(afrm, joints=jts)[0]
        lgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,awidth,gwidth = s2
            lasol = r.ik(afrm, joints=jts)[0]
            lgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, left arm')
            return

    q_start = r.get_joint_angles(joints='torso_arms')
    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    q_goal = r.get_joint_angles(joints='torso_arms')
    if pl.try_connect(q_start, q_goal, joints='torso_arms'):
        warn('collision free')
        sync(duration=tms['pick'])
    else:
        warn('detect arm collision')
        return

    r.set_joint_angles(rgsol, joints='rarm')
    r.set_joint_angles(lgsol, joints='larm')
    sync(duration=tms['pregrasp'])

    for w in [80,60,50,44,39,34]:
        r.grasp(w, hand='right')
        r.grasp(w, hand='left')
        sync(duration=0.1)

    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    sync(duration=tms['pregrasp'])

    r.set_joint_angle(0,-0.3)
    jts = 'rarm'
    r.set_joint_angles(r.ik(rwp, joints=jts)[0], joints=jts)
    jts = 'larm'
    r.set_joint_angles(r.ik(lwp, joints=jts)[0], joints=jts)
    sync(joints='torso_arms', duration=tms['transport'])

    r.set_joint_angle(0,-0.6)
    jts = 'rarm'
    x,y = pocketposs_dual[3] # right
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    jts = 'larm'
    x,y = pocketposs_dual[0] # left
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    sync(joints='torso_arms', duration=tms['transport'])

    P3 = env.get_object('P3')
    P0 = env.get_object('P0')
    while recognition:
        rpfrm = detect(hand='right', zmin=680, zmax=710, theta_constraint=[[0,pi/6],[5*pi/6,pi]])
        lpfrm = detect(hand='left', zmin=680, zmax=710, theta_constraint=[[0,pi/6],[5*pi/6,pi]])
        rpfrm.vec[2] = tblheight+15
        lpfrm.vec[2] = tblheight+15
        P3.locate(rpfrm, world=True)
        P0.locate(lpfrm, world=True)
        if linalg.norm(array(rpfrm.vec[:2]) - array(lpfrm.vec[:2])) > 90:
            break
        else:
            warn('too close places are detected')

    s1, s2 = place_plan(P3)
    jts = 'rarm'
    try:
        afrm,gfrm,rwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,rwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, right arm')
            return

    s1, s2 = place_plan(P0)
    jts = 'larm'
    try:
        afrm,gfrm,rwidth = s1
        lasol = r.ik(afrm, joints=jts)[0]
        lgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,rwidth = s2
            lasol = r.ik(afrm, joints=jts)[0]
            lgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, left arm')
            return

    q_start = r.get_joint_angles(joints='torso_arms')
    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    q_goal = r.get_joint_angles(joints='torso_arms')
    if pl.try_connect(q_start, q_goal, joints='torso_arms'):
        warn('collision free')
        sync(joints='torso_arms', duration=tms['place'])
    else:
        warn('detect arm collision')
        return

    r.set_joint_angles(rgsol, joints='rarm')
    r.set_joint_angles(lgsol, joints='larm')
    sync(joints='torso_arms', duration=tms['pregrasp'])

    for w in [38,40,46,80]:
        r.grasp(w, hand='right')
        r.grasp(w, hand='left')
        sync(duration=0.1)

    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    sync(duration=tms['pregrasp'])

    r.set_joint_angle(0,-0.3)
    jts = 'rarm'
    r.set_joint_angles(r.ik(rwp, joints=jts)[0], joints=jts)
    jts = 'larm'
    r.set_joint_angles(r.ik(lwp, joints=jts)[0], joints=jts)
    sync(joints='torso_arms', duration=tms['pick'])

    ####
    ####
    o1, o2 = choose_objs(1)

    r.set_joint_angle(0, 0)
    s1, s2 = grasp_plan(o1)
    jts = 'rarm'
    try:
        afrm,gfrm,awidth,gwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,awidth,gwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, right arm')
            return

    s1, s2 = grasp_plan(o2, long_side=True)
    jts = 'larm'
    try:
        afrm,gfrm,awidth,gwidth = s1
        lasol = r.ik(afrm, joints=jts)[0]
        lgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,awidth,gwidth = s2
            lasol = r.ik(afrm, joints=jts)[0]
            lgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, left arm')
            return

    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    sync(duration=tms['pick'])

    r.set_joint_angles(rgsol, joints='rarm')
    r.set_joint_angles(lgsol, joints='larm')
    sync(duration=tms['pregrasp'])

    for rw,lw in zip([80,60,50,44,39,34],[80,65,55,50,46,43]):
        r.grasp(rw, hand='right')
        r.grasp(lw, hand='left')
        sync(duration=0.1)

    r.set_joint_angles(rasol, joints='rarm')
    r.set_joint_angles(lasol, joints='larm')
    sync(joints='torso_arms', duration=tms['pregrasp'])

    r.set_joint_angle(0,-0.3)
    jts = 'rarm'
    r.set_joint_angles(r.ik(rwp, joints=jts)[0], joints=jts)
    jts = 'larm'
    r.set_joint_angles(r.ik(lwp, joints=jts)[0], joints=jts)
    sync(joints='torso_arms', duration=tms['transport'])

    jts = 'rarm'
    x,y = pocketposs_dual[2] # right
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    sync(joints='rarm', duration=tms['transport'])

    P2 = env.get_object('P2')
    if recognition:
        rpfrm = detect(hand='right', zmin=680, zmax=710, theta_constraint=[[0,pi/6],[5*pi/6,pi]])
        rpfrm.vec[2] = tblheight+15
        P2.locate(rpfrm, world=True)

    s1, s2 = place_plan(P2)
    jts = 'rarm'
    try:
        afrm,gfrm,rwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,rwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, right arm')
            return

    r.set_joint_angles(rasol, joints='rarm')
    sync(joints='rarm', duration=tms['place'])
    r.set_joint_angles(rgsol, joints='rarm')
    sync(joints='rarm', duration=tms['pregrasp'])

    for w in [38,40,46,80]:
        r.grasp(w, hand='right')
        sync(joints='rhand', duration=0.1)

    r.set_joint_angles(rasol, joints='rarm')
    sync(joints='rarm', duration=tms['pregrasp'])

    f = r.fk(arm='left')*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    r.set_joint_angles(r.ik(f,joints='larm')[0], joints='larm')
    f = FRAME(xyzabc=[lwp.vec[0],-160,lwp.vec[2],pi,0,pi/2])
    r.set_joint_angles(r.ik(f, joints='rarm')[0], joints='rarm')
    sync(joints='torso_arms', duration=tms['pick'])

    f = FRAME(xyzabc=[lwp.vec[0],-90,lwp.vec[2],pi,0,pi/2])
    r.set_joint_angles(r.ik(f, joints='rarm')[0], joints='rarm')
    sync(joints='rarm', duration=tms['pick'])
    r.grasp(34)
    sync(joints='rhand', duration=0.1)
    r.grasp(80, hand='left')
    sync(joints='lhand', duration=0.1)

    f = FRAME(xyzabc=[lwp.vec[0],-160,lwp.vec[2],pi,0,pi/2])
    r.set_joint_angles(r.ik(f, joints='rarm')[0], joints='rarm')
    sync(joints='rarm', duration=tms['transport'])

    jts = 'rarm'
    x,y = pocketposs_dual[1] # right
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    sync(joints='rarm', duration=tms['transport'])

    P1 = env.get_object('P1')
    if recognition:
        rpfrm = detect(hand='right', zmin=680, zmax=710, theta_constraint=[[0,pi/6],[5*pi/6,pi]])
        rpfrm.vec[2] = tblheight+15
        P1.locate(rpfrm, world=True)

    s1, s2 = place_plan(P1)
    jts = 'rarm'
    try:
        afrm,gfrm,rwidth = s1
        rasol = r.ik(afrm, joints=jts)[0]
        rgsol = r.ik(gfrm, joints=jts)[0]
    except:
        try:
            afrm,gfrm,rwidth = s2
            rasol = r.ik(afrm, joints=jts)[0]
            rgsol = r.ik(gfrm, joints=jts)[0]
        except:
            warn('ik failed, right arm')
            return

    r.set_joint_angles(rasol, joints='rarm')
    sync(joints='rarm', duration=tms['place'])
    r.set_joint_angles(rgsol, joints='rarm')
    sync(joints='rarm', duration=tms['pregrasp'])

    for w in [38,40,46,80]:
        r.grasp(w, hand='right')
        sync(duration=0.1)

    r.set_joint_angles(rasol, joints='rarm')
    sync(joints='rarm', duration=tms['pregrasp'])

    preapproach_dual()


def pick_and_place(n=1):
    preapproach(0)
    for i in range(n):
        f = detect()
        pick(f)
        transport(i)
        f = detect(zmin=tblheight-20, zmax=tblheight+30,
                   theta_constraint=[[0,pi/6],[5*pi/6,pi]])
        f.vec[0] -= 1.5
        place(f)
        if i == n-1:
            preapproach(0)
        else:
            preapproach(i+1)

pocketposs_dual = [(180,-240),(100,-240),
                   (180,-330),(100,-330)]

# detectposs_dual = [(160,-30),(160,150)]

def preapproach_dual():
    r.prepare(width=80)

    jts = 'rarm'
    x,y = detectposs_dual[0][0]
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)

    jts = 'larm'
    x,y = detectposs_dual[0][1]
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290,0,-pi/2,0])
    r.set_joint_angles(r.ik(f, jts)[0], joints=jts)

    sync(duration=tms['preapproach2'])


def dual_arm_pick_and_place_plan(oname00='A0', oname01='A2',
                                 pname00='P3', pname01='P0'):

    preapproach_dual()

    ## pick ##
    rofrm = detect(hand='right')
    lofrm = detect(hand='left')
    rofrm.vec[2] = 720
    lofrm.vec[2] = 722
    # adjust the positions of target objects
    A0 = env.get_object('A0')
    A0.locate(rofrm, world=True)
    A2 = env.get_object('A2')
    A2.locate(lofrm, world=True)

    q0 = r.get_joint_angles(joints='torso_arms')

    jts = 'rarm'
    parts = env.get_object(name=oname00)
    afrm,gfrm,handwidth = graspplan(objtype(parts), parts.where())

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,handwidth = request_next(afrm,gfrm,handwidth)

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    jts = 'larm'
    parts = env.get_object(name=oname01)
    afrm,gfrm,handwidth = graspplan(objtype(parts), parts.where())

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,handwidth = request_next(afrm,gfrm,handwidth)

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    r.set_joint_angles(gfrm_rarm_sol, joints='rarm')
    r.set_joint_angles(gfrm_larm_sol, joints='larm')
    sync(duration=1.0)
    r.grasp(width=34, hand='right')
    r.grasp(width=34, hand='left')
    sync(duration=1.0)
    grab(hand='right')
    grab(hand='left')
    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    sync(duration=1.0)

    ## transport ##
    q0 = r.get_joint_angles(joints='torso_arms')
    r.set_joint_angle(0, -0.6)

    jts = 'rarm'
    x,y = pocketposs_dual[3] # right
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    jts = 'larm'
    x,y = pocketposs_dual[0] # left
    f = FRAME(xyzabc=[x, y, tblheight+fsoffset+290, 0, -pi/2, 0])
    r.set_joint_angles(r.ik(f, joints=jts)[0], joints=jts)
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    ## place ##
    rpfrm = detect(hand='right', zmin=680)
    lpfrm = detect(hand='left', zmin=680)
    rpfrm.vec[2] = 724
    lpfrm.vec[2] = 726
    # adjust the positions of pockets
    P3 = env.get_object('P3')
    P3.locate(rpfrm, world=True)
    P0 = env.get_object('P0')
    P0.locate(lpfrm, world=True)

    q0 = r.get_joint_angles(joints='torso_arms')

    jts = 'rarm'
    plc = env.get_object(name=pname00)
    afrm,gfrm = placeplan(objtype(parts), plc.where())

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,_ = request_next(afrm,gfrm,0)

    try:
        afrm_rarm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_rarm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    jts = 'larm'
    plc = env.get_object(name=pname01)
    afrm,gfrm = placeplan(objtype(parts), plc.where())

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        afrm,gfrm,_ = request_next(afrm,gfrm,0)

    try:
        afrm_larm_sol = r.ik(afrm, joints=jts)[0]
        gfrm_larm_sol = r.ik(gfrm, joints=jts)[0]
    except:
        warn('ik solution not found: %s'%jts)
        return

    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    q1 = r.get_joint_angles(joints='torso_arms')
    traj = pl.make_plan(q0, q1, joints='torso_arms')
    exec_traj(traj, joints='torso_arms')

    r.set_joint_angles(gfrm_rarm_sol, joints='rarm')
    r.set_joint_angles(gfrm_larm_sol, joints='larm')
    sync(duration=0.5, joints='torso_arms')
    r.grasp(width=80, hand='right')
    r.grasp(width=80, hand='left')
    sync(duration=0.5)
    release(hand='right')
    release(hand='left')
    r.set_joint_angles(afrm_rarm_sol, joints='rarm')
    r.set_joint_angles(afrm_larm_sol, joints='larm')
    sync(duration=0.5)

    go_prepare_pose()


# plt = env.get_object('pallete0')

# A0 = env.get_object('A0')
# f = A0.rel_trans
# f.vec[1] += 40
# A0.locate(f)
# env.delete_object('A1')
# env.delete_object('A3')
# env.delete_object('B0')
# env.delete_object('B1')


pose1 = {'rarm': FRAME(xyzabc=[230, -200, 1000-1.5, 0, -pi/2, -pi/2]),
         'larm': FRAME(xyzabc=[185, 200, 1000, 0, -pi/2, pi/2])}

pose2 = {'rarm': FRAME(xyzabc=[140, -200, 1000-1.5, 0, -pi/2, -pi/2]),
         'larm': FRAME(xyzabc=[185, 200, 1000, 0, -pi/2, pi/2])}
