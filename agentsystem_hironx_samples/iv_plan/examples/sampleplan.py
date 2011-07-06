# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *
from scene_objects import *

# r.add_collision_object(env.get_object('wall0'))
# r.add_collision_object(env.get_object('wall1'))

def demo_plan():
    def grasp_width(obj):
        if obj.vbody.__class__ == visual.cylinder:
            return obj.vbody.radius*2
        else:
            return obj.vbody.size[1]
        
    prepare_right()
    q_pre = r.get_arm_joint_angles()

    pltfrm = env.get_object('pallete').where()
    holefrm = pltfrm*FRAME(xyzabc=[65,41,0,0,0,0])

    # parameters
    appdist = {'partsA': 50, 'partsB': 100}
    grasp_zoffset = {'partsA': 0, 'partsB': 85}

    for tgtnm in ['partsA', 'partsB']:
        # grasp sequence
        pt = env.get_object(tgtnm)
        ptfrm = pt.where()
        ptfrm.vec[2] += grasp_zoffset[tgtnm]
        asols,gsols = pl.reaching_plan(ptfrm, use_waist=False)
    
        move_arm(asols[0])
        r.set_arm_joint_angles(gsols[0])
        grasp(width=grasp_width(pt), name=tgtnm)
        remove_hand_collision(tgtnm)
        grasped_height = ptfrm.vec[2]

        if tgtnm == 'partsB':
            remove_objects_collision('partsA', 'partsB')

        # place sequence
        tgtfrm = FRAME(holefrm)
        tgtfrm.vec[2] = grasped_height
        asols,gsols = pl.reaching_plan(tgtfrm, use_waist=False, approach_distance=appdist[tgtnm])
        move_arm(asols[0])
        r.set_arm_joint_angles(gsols[0])
        release(name = tgtnm)
        add_hand_collision(tgtnm)
    
        # go back to prepare pose
        move_arm(q_pre)

def demo_plan1():
    p_start = FRAME(mat=[[0, 0, -1], [0, 1, 0], [1, 0, 0]],
                    vec=[210.0, -300.0, 880.0])
    q_start = r.ik(p_start)[0]
    p_target = FRAME(mat=[[0, 0, -1], [0, 1, 0], [1, 0, 0]],
                     vec=[210.0, -50.0, 880.0])
    q_target = r.ik(p_target)[0]
    traj = pl.make_plan(q_start, q_target)
    if traj == None:
        return
    show_traj(traj)
    opttraj = pl.optimize_trajectory(traj)
    show_traj(opttraj)
    return traj, opttraj

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

def play_traj(traj, duration=0.5):
    '''function to execute a planner-generated trajectory'''
    avecs = [x.avec for x in traj]
    for avec in avecs:
        r.set_arm_joint_angles(avec)
        sync(duration=duration)

def setup_ac_scene():
    tbltop = env.get_object('table top')
    thickness = tbltop.vbody.size[2]
    name = 'pallete'
    plt = env.get_object('pallete')
    if not plt:
        newplt = env.eval_sctree(pallete(name))
        x,y,theta = -180, -370, 0
        reltf = FRAME(xyzabc=[x,y,(thickness+0)/2,0,0,pi/2])
        env.insert_object(newplt, reltf, tbltop)

    name = 'partsA'
    x,y,theta = -200, 100, 0
    if plt:
        pt = env.get_object(name)
        reltf = FRAME(xyzabc=[x,y,(thickness+pt.vbody.size[2])/2,
                              0,0,theta])
        pt.unfix()
        pt.affix(tbltop, reltf)
    else:
        pt = env.eval_sctree(partsA(name))
        reltf = FRAME(xyzabc=[x,y,(thickness+pt.vbody.size[2])/2,
                              0,0,theta])
        env.insert_object(pt, reltf, tbltop)

    name = 'partsB'
    x,y,theta = -200, -50, 0
    if plt:
        pt = env.get_object(name)
        reltf = FRAME(xyzabc=[x,y,thickness/2.0,0,0,theta])
        pt.unfix()
        pt.affix(tbltop, reltf)
    else:
        pt = env.eval_sctree(partsB(name))
        reltf = FRAME(xyzabc=[x,y,thickness/2.0,0,0,theta])
        env.insert_object(pt, reltf, tbltop)

    if not plt:
        setup_cobjs()

def setup_cobjs():
    pltobjs = [x for x in env.get_objects() if re.match('pallete side.*', x.name)]
    ptA = env.get_object('partsA')
    ptB = env.get_object('partsB')
    for o in pltobjs:
        r.add_collision_object(o)
        r.add_collision_pair(ptA, o)
        r.add_collision_pair(ptB, o)
    r.add_collision_object(ptA)
    r.add_collision_object(ptB)
    r.add_collision_pair(ptA, ptB)

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

def prepare_right():
    r.set_joint_angles(r.poses['prepare_right'])

def move_arm(q_target):
    q_start = r.get_arm_joint_angles()
    traj = pl.make_plan(q_start, q_target)
    if traj == None:
        return
    opttraj = pl.optimize_trajectory(traj)
    show_traj(opttraj)
    return opttraj

# def demo_plan():
#     prepare_right()
#     q_pre = r.get_arm_joint_angles()

#     pltfrm = env.get_object('pallete').where()
#     holefrms = [pltfrm*FRAME(xyzabc=[65,41,0,0,0,0]),
#                 pltfrm*FRAME(xyzabc=[65,-41,0,0,0,0]),
#                 pltfrm*FRAME(xyzabc=[-65,41,0,0,0,0]),
#                 pltfrm*FRAME(xyzabc=[-65,-41,0,0,0,0])]

#     objs = detect()
#     for tgtnm in ['partsA', 'partsB']:
#         pt = env.get_object(tgtnm)

setup_ac_scene()
