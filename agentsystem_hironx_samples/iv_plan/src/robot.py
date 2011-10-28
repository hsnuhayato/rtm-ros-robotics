# -*- coding: utf-8 -*-

import os
import re
import operator
from subprocess import Popen, PIPE

from numpy import *
from ivutils import *
from viewer import *
import wrl_loader

from pqp_if import *
import libik_hiro as ikfast

def get_AABB(vs, padding=8.0):
    xlb = ylb = zlb = inf
    xub = yub = zub = -inf

    if vs.__class__ == list:
        for v in vs:
            if v[0] < xlb:
                xlb = v[0]
            if v[0] > xub:
                xub = v[0]
            if v[1] < ylb:
                ylb = v[1]
            if v[1] > yub:
                yub = v[1]
            if v[2] < zlb:
                zlb = v[2]
            if v[2] > zub:
                zub = v[2]
    else:
        if vs.vbody.__class__ == visual.cylinder:
            r = vs.vbody.radius
            z = vs.vbody.axis[2]
            xlb = -r
            xub = r
            ylb = -r
            yub = r
            zlb = 0
            zub = z
        else:
            warn('unknown shape')
            return None
    return [[xlb-padding,xub+padding],
            [ylb-padding,yub+padding],
            [zlb-padding,zub+padding]]

def gen_collision_body(obj):
    tris = [(0,1,2), (2,3,0),
            (0,4,1), (4,5,1),
            (1,5,2), (5,6,2),
            (2,3,6), (3,7,6),
            (0,4,7), (0,7,3),
            (4,5,6), (4,6,7)]

    def gen_cbody_from_AABB(aabb):
        [[x0,x1],[y0,y1],[z0,z1]] = aabb
        lx = x1-x0
        ly = y1-y0
        lz = z1-z0
        pts = [[x0+lx,y0,   z0+lz],
               [x0+lx,y0+ly,z0+lz],
               [x0,   y0+ly,z0+lz],
               [x0,   y0,   z0+lz],
               [x0+lx,y0,   z0   ],
               [x0+lx,y0+ly,z0   ],
               [x0,   y0+ly,z0   ],
               [x0,   y0,   z0   ]]

        b = cpqp.PQP_Model()
        b.BeginModel(8)
        for i in range(len(tris)):
            tri = tris[i]
            b.AddTri(pts[tri[0]],pts[tri[1]],pts[tri[2]],i)
        b.EndModel()
        b.MemUsage(1)
        return b

    def gen_cbody(obj):
        if obj.vbody.__class__ == visual.box:
            x0 = obj.vbody.x - obj.vbody.length/2.0
            y0 = obj.vbody.y - obj.vbody.height/2.0
            z0 = obj.vbody.z - obj.vbody.width/2.0
            x1 = obj.vbody.x + obj.vbody.length/2.0
            y1 = obj.vbody.y + obj.vbody.height/2.0
            z1 = obj.vbody.z + obj.vbody.width/2.0
            return gen_cbody_from_AABB([[x0,x1],[y0,y1],[z0,z1]])
        else:
            #warn(str(obj.vbody.__class__)+' is not supported for collision body')
            return gen_cbody_from_AABB(get_AABB(obj))

    def gen_cbody_link(l, simple=False):
        pts = []
        for tf,shp in l.shapes:
            # print shp.vbody.__class__
            if shp.vbody.__class__ == visual.faces:
                # transform all the 'pos' and add them to pts
                for pos in shp.vbody.pos:
                    # print pos
                    pts.append(pos)

        if len(pts) == 0:
            return
        if simple:
            aabb = get_AABB(pts)
            print l.name,
            print aabb
            return gen_cbody_from_AABB(aabb)
        else:
            b = cpqp.PQP_Model()
            b.BeginModel(8)
            for i in range(len(pts)/3):
                b.AddTri(pts[3*i],pts[3*i+1],pts[3*i+2],i)
            b.EndModel()
            b.MemUsage(1)
            return b

    if obj.__class__ == LinkObject:
        return gen_cbody_link(obj)
    elif obj.__class__ == PartsObjectWithName:
        try:
            return gen_cbody(obj)
        except:
            return None
    elif obj.__class__ == KinbodyObject:
        return gen_cbody(obj)        

def in_collision_pair(obj1, obj2, cache):
    if obj1.cb == None or obj2.cb == None:
        return False

    try:
        R1,T1 = cache[obj1]
    except:
        f1 = obj1.where()
        R1 = array(f1.mat[:3][:3])
        T1 = array(f1.vec)
        cache[obj1] = (R1,T1)

    try:
        R2,T2 = cache[obj2]
    except:
        f2 = obj2.where()
        R2 = array(f2.mat[:3][:3])
        T2 = array(f2.vec)
        cache[obj2] = (R2,T2)

    cres = collide(R1, T1, obj1.cb, R2, T2, obj2.cb, cpqp.contact_type.first)

    if cres.NumPairs() > 0:
        print '%s <=> %s'%(obj1.name, obj2.name)
        # print_collide_result(cres)
        return True
    else:
        return False

def in_collision_pair_parts(obj1, obj2, cache):
    if in_collision_pair(obj1, obj2, cache):
        return True
    for cobj1 in obj1.children:
        if isinstance(cobj1, KinbodyObject):
            for cobj2 in obj2.children:
                if in_collision_pair(cobj1, cobj2, cache):
                    return True
            if in_collision_pair(cobj1, obj2, cache):
                return True
    for cobj2 in obj2.children:
        if isinstance(cobj2, KinbodyObject):
            if in_collision_pair(cobj2, obj1, cache):
                return True
    return False


class VRobot(JointObject):
    def __init__(self, wrldir, scale, robotname):
        JointObject.__init__(self, 0, robotname, [0,0,1], FRAME())
        # self.wrldir = os.getcwd() + re.sub('[^\/]*$', '', wrlfile)
        # print ('wrl directory = ' + self.wrldir)
        loader = wrl_loader.WrlLoader()
        name, joints, links = loader.load(wrldir)

        self.joints = joints
        self.links = links
        self.link = LinkObject(self, name='BASE_Link')
        self.links.insert(0, self.link)
        joints[0].parent.affix(self, FRAME(vec=VECTOR(vec=[0,0,800])))

        self.gen_link_collision_body()
        self.cobj_pairs = []
        self.sensors = []

        self.grabbed_obj = {'right': None, 'left': None}

        self.init_clink_pairs()
        self.reset_pose()

    def __repr__(self):
        return '<VRobot %s>'%self.name

    def __str__(self):
        return self.__repr__()

    def add_collision_object(self, obj):
        lnknames = ['ARM_JOINT2_Link', 'ARM_JOINT3_Link',
                    'ARM_JOINT4_Link', 'ARM_JOINT5_Link',
                    'HAND_JOINT0_Link', 'HAND_JOINT1_Link',
                    'HAND_JOINT2_Link', 'HAND_JOINT3_Link']
        for lnknm in ['R'+s for s in lnknames]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        for lnknm in ['L'+s for s in lnknames]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)

    def grasp_collision_object(self, obj, hand='right'):
        for lnknm in ['WAIST_Link', 'CHEST_JOINT0_Link',
                      'HEAD_JOINT0_Link', 'HEAD_JOINT1_Link']:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        prefix1 = 'L' if hand == 'right' else 'R'
        prefix2 = 'R' if hand == 'right' else 'L'
        for lnknm in [prefix1+s for s in ['ARM_JOINT0_Link',
                                           'ARM_JOINT1_Link',
                                           'HAND_JOINT0_Link',
                                           'HAND_JOINT1_Link',
                                           'HAND_JOINT2_Link',
                                           'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)
        for lnknm in [prefix2+s for s in ['HAND_JOINT0_Link',
                                            'HAND_JOINT1_Link',
                                            'HAND_JOINT2_Link',
                                            'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)

    def release_collision_object(self, obj, hand='right'):
        for lnknm in ['WAIST_Link', 'CHEST_JOINT0_Link',
                      'HEAD_JOINT0_Link', 'HEAD_JOINT1_Link']:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)
        prefix1 = 'L' if hand == 'right' else 'R'
        prefix2 = 'R' if hand == 'right' else 'L'
        for lnknm in [prefix1+s for s in ['ARM_JOINT0_Link',
                                           'ARM_JOINT1_Link',
                                           'HAND_JOINT0_Link',
                                           'HAND_JOINT1_Link',
                                           'HAND_JOINT2_Link',
                                           'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.remove_collision_pair(lnk, obj)
        for lnknm in [prefix2+s for s in ['HAND_JOINT0_Link',
                                          'HAND_JOINT1_Link',
                                          'HAND_JOINT2_Link',
                                          'HAND_JOINT3_Link']]:
            lnk = self.get_link(lnknm)
            self.add_collision_pair(lnk, obj)

    def add_collision_pair(self, obj1, obj2):
        obj1.cb = gen_collision_body(obj1)
        for cobj in obj1.children:
            if isinstance(cobj, KinbodyObject) and (not cobj.cb):
                cobj.cb = gen_collision_body(cobj)
        obj2.cb = gen_collision_body(obj2)
        for cobj in obj2.children:
            if isinstance(cobj, KinbodyObject) and (not cobj.cb):
                cobj.cb = gen_collision_body(cobj)

        if not (obj1,obj2) in self.cobj_pairs:
            self.cobj_pairs.append((obj1, obj2))

    def remove_collision_pair(self, obj1, obj2):
        self.cobj_pairs = [(x,y) for x,y in self.cobj_pairs if not ((x == obj1 and y == obj2) or (x == obj2 and y == obj1))]

    def init_clink_pairs(self):
        blacklist = [(0,2),(0,3),(0,4),(0,5),(0,9),(0,10),(0,11),
                     (2,3),(2,9),
                     (6,8),(6,15),(6,17),
                     (7,15),(7,17),
                     (8,15),(8,17),
                     (12,14),(12,19),(12,21),
                     (13,19),(13,21),
                     (14,19),(14,21),
                     (15,17),(15,18),(16,18),
                     (19,21),(19,22),(20,22)
                     ]

        self.clink_pairs = []
        n = len(self.joints)
        for i in range(n):
            for j in range(i+2, n):
                if not (i, j) in blacklist:
                    self.clink_pairs.append((self.joints[i].link,
                                             self.joints[j].link))

    def in_collision(self, check_all=False):
        cache = {}
        for l1,l2 in self.clink_pairs:
            if in_collision_pair_parts(l1, l2, cache):
                return True
        for obj1, obj2 in self.cobj_pairs:
            if in_collision_pair_parts(obj1, obj2, cache):
                return True
        return False

    def gen_link_collision_body(self):
        for lnk in self.get_links():
            lnk.cb = gen_collision_body(lnk)

    def set_joint_angles(self, ths, joints='all', flush=True):
        if joints == 'rarm':
            js = self.joints[3:9]
        elif joints == 'larm':
            js = self.joints[9:15]
        elif joints == 'torso_rarm':
            js = [self.joints[0]]+self.joints[3:9]
        elif joints == 'torso_larm':
            js = [self.joints[0]]+self.joints[9:15]
        elif joints == 'torso_arms':
            js = [self.joints[0]]+self.joints[3:15]
        else:
            js = self.joints

        for j,th in zip(js,ths):
            j.angle = th

        if flush:
            self.refresh()

    def set_joint_angle(self, id, th, flush=True):
        self.joints[id].angle = th
        if flush:
            self.refresh()

    def refresh(self):
        for j in self.joints:
            newth = j.angle
            jaxis = j.jaxis
            ltrans = j.reltrans
            obj = j
            jtrans = FRAME(mat=MATRIX(axis=jaxis,angle=newth))
            reltrans = ltrans * jtrans
            obj.set_trans(reltrans)

    def get_joint_angles(self, joints='all'):
        if joints == 'rarm':
            js = self.joints[3:9]
        elif joints == 'larm':
            js = self.joints[9:15]
        elif joints == 'torso_rarm':
            js = [self.joints[0]]+self.joints[3:9]
        elif joints == 'torso_larm':
            js = [self.joints[0]]+self.joints[9:15]
        elif joints == 'torso_arms':
            js = [self.joints[0]]+self.joints[3:15]
        else:
            js = self.joints
        return [j.angle for j in js]

    def get_joint_angle(self, id):
        return self.joints[id].angle

    def reset_pose(self):
        self.set_joint_angles(self.poses['init'])

    def prepare(self, width=None):
        self.set_joint_angles(self.poses['prepare'])
        if width:
            self.grasp(width=width)
            self.grasp(width=width, hand='left')

    def prepare_right(self):
        self.set_joint_angles(self.poses['prepare_right'])

    def go_pos(self, x, y, theta):
        self.set_trans(FRAME(xyzabc=[x,y,0,0,0,theta]))

    def get_link(self, name):
        return [x for x in self.get_links() if x.name == name][0]

    def get_joint(self, name):
        return [x for x in self.get_joints() if x.name == name][0]

    def get_sensor(self, name):
        return [x for x in self.get_sensors() if x.name == name][0]

    def get_links(self):
        return self.links

    def get_joints(self):
        return self.joints

    def get_sensors(self):
        return self.sensors

class VPA10(VRobot):
    def __init__(self,
                 wrldir = ivpkgdir+'/iv_plan/externals/models/PA10_2/',
                 scale = 1000.0,
                 name = 'pa10'):
        self.poses = { 'reset' : zeros(9),
                       'manip' : array([0,-0.4,0,1.2,0,1,0]),
                       'pickup' : array([1.4,0.3,0,1.2,0,1.6,0]),
                       'undertable' : array([1.5,0.3,0,2.2,0,0.5,0])
                       }

        VRobot.__init__(self, wrldir, scale, name)


class VRH2(VRobot):
    def __init__(self,
                 wrldir = ivpkgdir+'/iv_plan/externals/models/RH2/',
                 scale = 1000.0,
                 name = 'rh2'):

        self.poses = { 'reset' : zeros(1) }

        VRobot.__init__(self, wrldir, scale, name)


import hironx_params

class VHIRONX(VRobot):
    def __init__(self,
                 wrldir = ivpkgdir+'/iv_plan/externals/models/HIRO_110603/',
                 scale = 1000.0,
                 name = 'HIRO-NX'):

        # predefined poses
        self.poses = hironx_params.poses
        self.hand_poses = hironx_params.hand_poses

        # fixed transforms
        self.Thd_leye = hironx_params.Thd_leye
        self.Trh_cam = hironx_params.Trh_cam
        self.Tlh_cam = hironx_params.Tlh_cam
        self.Thd_kinectrgb = hironx_params.Thd_kinectrgb
        self.Thd_kinectdepth = hironx_params.Thd_kinectdepth
        self.Trwrist_ef = hironx_params.Trwrist_ef
        self.Tlwrist_ef = hironx_params.Tlwrist_ef

        self.Twrist_ef = hironx_params.Trwrist_ef # deprecated

        self.Tikoffset = hironx_params.Tikoffset

        VRobot.__init__(self, wrldir, scale, name)

        # safe(soft) joint limits
        for i,lims in enumerate([deg2rad(x) for x in [(-90,90),(-70,70),(-20,70),
                                                      (-80,80),(-86,29),(-143,-12),(-86,86),(-95,95),(-110,110),
                                                      (-80,80),(-86,29),(-143,-12),(-86,86),(-95,95),(-110,110),
                                                      (-109,68),(-150,90),(-109,68),(-150,90),
                                                      (-109,68),(-150,90),(-109,68),(-150,90)]]):
            j = self.joints[i]
            j.sllimit,j.sulimit = lims

        # maximum workspace movement in [mm]
        for j,w in zip(self.joints, [800,200,200,
                                     700,650,320,120,140,50,
                                     700,650,320,120,140,50,
                                     20,20,20,20,
                                     20,20,20,20]):
            j.weight = w

        # attach sensors
        rhandcam = SensorObject(name='rhandcam')
        rhandcam.affix(self.get_joint('RARM_JOINT5'), self.Trh_cam)
        self.sensors.append(rhandcam)
        lhandcam = SensorObject(name='lhandcam')
        lhandcam.affix(self.get_joint('LARM_JOINT5'), self.Tlh_cam)
        self.sensors.append(lhandcam)
        kinectdepth = SensorObject(name='kinectdepth')
        kinectdepth.affix(self.get_joint('HEAD_JOINT1'), self.Thd_kinectdepth)
        self.sensors.append(kinectdepth)

    def set_arm_joint_angles(self, ths, arm='right', flush=True):
        '''@rtc_interface: void SetArmJointAngles(in DoubleSequence, in String, in bool)
        '''
        
        rarm = self.check_right_or_left(arm)
        
        if len(ths) != 6:
            print "the length of joint angles is wrong"
            return
        
        if rarm:
            jointid_offset = 3
        else:
            jointid_offset = 9

        for i in range(len(ths)):            
            self.set_joint_angle(i+jointid_offset, ths[i], flush=False)
        if flush:
            self.refresh()

    def get_arm_joint_angles(self, arm='right'):
        rarm = self.check_right_or_left(arm)
        
        if rarm:
            return self.get_joint_angles()[3:9]
        else:
            return self.get_joint_angles()[9:15]            

    def get_hand_joint_angles(self, hand='right'):
        rhand = self.check_right_or_left(hand)

        if rhand:
            return self.get_joint_angles()[15:19]
        else:
            return self.get_joint_angles()[19:23]

    def set_hand_joint_angles(self, angles, hand='right'):
        rhand = self.check_right_or_left(hand)
        if len(angles) != 4:
            print "the length of joint angles is wrong"
            return
        
        js = self.get_joint_angles()
        if rhand:
            js[15:19] = angles
        else:
            js[19:23] = angles
        self.set_joint_angles(js)

    def fk_fast_rarm(self, ths, scl=1e+3):
        rot,trans = ikfast.fk(ths)
        m = MATRIX(mat=rot)
        v = VECTOR(vec=[scl*x for x in trans])
        return FRAME(mat=m, vec=v)
    
    def fk(self, arm='right', scl=1e+3):
        rarm = self.check_right_or_left(arm)
        
        if rarm:
            ths = self.get_joint_angles()[3:9]
            f = self.fk_fast_rarm(ths)
            f = f*(-self.Tikoffset) ###
            Twc = self.links[2].where()
            return Twc*f
        else:
            ths = self.get_joint_angles()[9:15]
            for i in [0,3,5]:
                ths[i] = -ths[i]
            f = self.fk_fast_rarm(ths)
            f = f*(-self.Tikoffset) ###
            x,y,z = f.vec
            a,b,c = f.mat.abc()
            f = FRAME(xyzabc=[x,-y,z,-a,b,-c])
            Twc = self.links[2].where()
            return Twc*f

    def ik_fast_rarm(self, efframe, scl=1e-3):
        efframe = efframe*self.Tikoffset ###
        return ikfast.ik(efframe.mat, [scl * x for x in efframe.vec])

    def ik(self, frms, joints='rarm', scl=1e-3):
        def reverse_frame(frm):
            x,y,z = frm.vec
            a,b,c = frm.mat.abc()
            return FRAME(xyzabc=[x,-y,z,-a,b,-c])

        arm,use_waist = parse_joints_flag(joints)

        rarm = self.check_right_or_left(arm)

        if frms.__class__ == FRAME:
            frms = [frms]

        # evaluate only arm pose
        # q_reference = self.poses['prepare'][3:9]
        q_reference = self.get_joint_angles()[3:9]

        if use_waist:
            th_orig = self.get_joint_angle(0) # save the waist yaw angle
            waist_yaw_samples = linspace(-1.0, 1.0, 11)
            sols = []
            for th in waist_yaw_samples:
                self.set_joint_angle(0, th)
                Twc = self.links[2].where()
                frms2 = [(-Twc)*frm for frm in frms]
                if not rarm:
                    frms2 = map(reverse_frame, frms2)

                sols += [(th,x) for x in reduce(operator.add,
                                                map(lambda frm: self.ik_fast_rarm(frm),
                                                    frms2))]
            sols2 = self.eval_solutions(sols, q_reference, use_waist=use_waist)
            if not rarm:
                for sol in sols2:
                    for i in [0,3,5]:
                        sol[1][i] = -sol[1][i]

            self.set_joint_angle(0, th_orig) # restore the waist yaw angle

            sols3 = []
            for th,avec in sols2:
                avec.insert(0, th)
                sols3.append(avec)
            return sols3

        else:
            Twc = self.links[2].where()
            frms = map(lambda frm: (-Twc)*frm, frms)
            if not rarm:
                frms = map(reverse_frame, frms)

            sols = reduce(operator.add,
                          map(lambda frm: self.ik_fast_rarm(frm),
                              frms))
            sols2 = self.eval_solutions(sols, q_reference, use_waist=use_waist)
            if not rarm:
                for sol in sols2:
                    for i in [0,3,5]:
                        sol[i] = -sol[i]
            return sols2

    def weighted_qdist(self, q1, q2):
        return weighted_L1dist(q1, q2, [1.0, 0.8, 0.3, 0.1, 0.1, 0.02])
    
    def within_joint_limit(self, q):
        for i in range(6):
            jnt = self.joints[i+3]
            if q[i] < jnt.sllimit or q[i] > jnt.sulimit:
                # print 'Limits: %d,%f,%f,%f'%(i+3,q[i],jnt.sllimit,jnt.sulimit)
                return False
        return True

    def eval_solutions(self, sols, q_reference, use_waist=False):
        if use_waist:
            sols2 = [x for x in sols if self.within_joint_limit(x[1])]
            sols2.sort(lambda q1,q2: cmp(self.weighted_qdist(q_reference, q1[1]),
                                         self.weighted_qdist(q_reference, q2[1])))
        else:
            sols2 = [x for x in sols if self.within_joint_limit(x)]
            sols2.sort(lambda q1,q2: cmp(self.weighted_qdist(q_reference, q1),
                                         self.weighted_qdist(q_reference, q2)))

        # print 'feasible solutions'
        # print sols2
        return sols2

    def check_right_or_left(self, arm):
        if arm == 'right':
            return True
        else:
            return False

    def grasp(self, width, affixobj=False, hand='right'):
        th = asin(((width/2.0) - 15) / 42)
        js = [th, -th, -th, th]
        self.set_hand_joint_angles(js, hand=hand)

