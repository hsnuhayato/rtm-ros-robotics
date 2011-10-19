# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from numpy import *
import sys
import time

from demo_common import *

# definitions of piece geometry
def cube(name, color, l):
    return {'type':'kinbody', 'name':name, 'shape':'box',
            'color':color, 'material':None,
            'dimension':[l-1, l-1, l-1], 'children':[]}

def generate_piece(nametag, color, cubeposes, l=30.0):
    l2 = l/2
    xyzabcs = cubeposes
    cbs = [(cube(name=nametag+str(i), color=color, l=l),xyzabc) for i,xyzabc in enumerate(xyzabcs)]
    return env.eval_sctree({'type':'parts', 'name':'piece-'+nametag, 'shape':None, 'children':cbs})

init_frames = {}

def setup_puzzle_scene():
    l = 30.0
    tbl = env.get_object('table')
    tbltop = env.get_object('table top')
    ttz = tbltop.where().vec[2] + tbltop.vbody.size[2]/2
    z = ttz + l/2

    # generate 7 pieces
    # dark green piece
    env.insert_object(generate_piece('green', (0.0, 0.392, 0.0),
                                     [(0,0,0,0,0,0),(0,0,l,0,0,0),(l,0,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-310,-80,z,0,0,-pi/2]), tbl)
    # yellow green piece
    env.insert_object(generate_piece('yellow-green', (0.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(l,0,0,0,0,0)]),
                      FRAME(xyzabc=[-180,0,z+l,-pi/2,pi,0]), tbl)
    # red piece
    env.insert_object(generate_piece('red', (1.0, 0.0, 0.0),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(2*l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-300,-200,z+l,-pi/2,0,0]), tbl)
    # purple piece
    env.insert_object(generate_piece('purple', (0.6, 0.196, 0.8),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(0,0,l,0,0,0)]),
                      FRAME(xyzabc=[-180,-150,z,0,-pi/2,0]), tbl)
    # aqua piece
    env.insert_object(generate_piece('aqua', (0.0, 1.0, 1.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(0,2*l,0,0,0,0),(l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-350,-300,z,pi/2,0,0]), tbl)
    # brown piece
    env.insert_object(generate_piece('brown', (0.545, 0.27, 0.075),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-250,-300,z,0,0,0]), tbl)
    # yellow piece
    env.insert_object(generate_piece('yellow', (1.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(0,2*l,0,0,0,0),(l,0,0,0,0,0)]),
                      FRAME(xyzabc=[-350,0,z,0,0,-pi/2]), tbl)

    # remember the initial positions of the pieces in world frame
    for p in env.get_objects('^piece'):
        init_frames[p.name] = FRAME(p.where())

    # register collision pairs and graspable objects
    ps = env.get_objects('^piece')
    for p in ps:
        r.add_collision_object(p)
        for p2 in ps:
            if p != p2:
                r.add_collision_pair(p, p2)
        env.add_collidable_object(p)
    env.add_collidable_object(env.get_object('table'))


def reset_puzzle():
    for p in env.get_objects('^piece'):
        p.locate(init_frames[p.name], world=True)


def grasp_plan(obj):
    '''given an object, compute handlink frame and distance between fingers to pick the piece'''
    l = 30
    d = 30

    w1 = (80, 26)
    w2 = (100, 56)
    w3 = (110, 86)

    grspposs = {
        'piece-green': (FRAME(xyzabc=[0,0,l,0,0,pi/2]), w1),
        'piece-yellow-green': (FRAME(xyzabc=[0,0,0,-pi/2,pi,0]), w1),
        'piece-red': (FRAME(xyzabc=[0.5*l,0,0,pi/2,0,pi/2]), w2),
        'piece-purple': (FRAME(xyzabc=[l,0,0,0,pi/2,pi/2]), w1),
        'piece-aqua': (FRAME(xyzabc=[0,2*l,0,-pi/2,0,-pi/2]), w1),
        'piece-brown': (FRAME(xyzabc=[0.5*l,l,0,0,0,pi/2]), w2),
        'piece-yellow': (FRAME(xyzabc=[l,l,0,0,0,pi]), w3)
    }

    f, (awidth,gwidth) = grspposs[obj.name]

    gfrm = obj.where()*f*(-r.Trwrist_ef)
    afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
    return afrm,gfrm,awidth,gwidth


def place_plan(obj, hand='right'):
    l = 30
    d = 30

    tbl = env.get_object('table')
    tbltop = env.get_object('table top')
    ttz = tbltop.where().vec[2] + tbltop.vbody.size[2]/2
    z = ttz + l/2

    # reference frame
    rfrm = FRAME(xyzabc=[190,-80,z,0,0,-pi/2])

    plcposs = {
        'piece-green':FRAME(xyzabc=[0,0,0,0,0,0]),
        'piece-yellow-green':FRAME(xyzabc=[0,2*l,l,-pi/2,pi/2,0]),
        'piece-red':FRAME(xyzabc=[l,0,l,-pi/2,-pi/2,0]),
        'piece-purple':FRAME(xyzabc=[2*l,2*l,0,pi/2,0,pi/2]),
        'piece-aqua':FRAME(xyzabc=[2*l,0,0,pi/2,pi/2,0]),
        'piece-brown':FRAME(xyzabc=[2*l,l,2*l,0,0,pi/2]),
        'piece-yellow':FRAME(xyzabc=[0,0,2*l,0,0,0])
    }

    handfrm = r.get_link(hand[0].upper()+'ARM_JOINT5_Link').where()
    objfrm = obj.where()

    gfrm = rfrm*plcposs[obj.name]*(-objfrm)*handfrm
    afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
    width = 80.0 # hand width when releasing a piece
    return afrm,gfrm,width

def pick_piece(obj, jts='rarm', hand='right'):
    # determine grasp position & hand width
    afrm,gfrm,awidth,gwidth = grasp_plan(obj)

    r.grasp(awidth, hand=hand)

    # initial configuration
    q0 = r.get_joint_angles(joints=jts)
    # compute goal configuration by solving inverse kimematics
    q1 = r.ik(afrm, joints=jts)[0]

    # connect 2 configurations by RRT-connect
    traj = pl.make_plan(q0, q1, joints=jts)
    # execute the trajectory
    exec_traj(traj, joints=jts)

    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    r.grasp(gwidth, hand=hand)

def place_piece(obj, p=FRAME(xyzabc=[0,0,0,0,0,0]), jts='rarm', hand='right'):
    afrm,gfrm,width = place_plan(obj, hand)
    q0 = r.get_joint_angles(joints=jts)
    q1 = r.ik(afrm, joints=jts)[0]
    traj = pl.make_plan(q0, q1, joints=jts)
    exec_traj(traj, joints=jts)
    
    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    r.grasp(width, hand=hand)

def demo(jts='rarm'):
    hand = 'right' if re.match('.*rarm', jts) else 'left'
    
    cl = ['green','yellow-green','red','purple','aqua','brown','yellow']
    for c in cl:
        obj = env.get_object('piece-'+c)
        r.grasp(80, hand=hand)
        pick_piece(obj, jts=jts, hand=hand)
        grab(hand=hand)
        place_piece(obj, jts=jts, hand=hand)
        release()
    r.prepare()


import discrete_puzzle
def parse_solution(sol, step=True):
    def aux(action):
        l = 30.0
        n,p = action
        pos, (ex,ey) = p.qs[n][2]
        ez = cross(ex, ey)
        return p.col, FRAME(mat=transpose(array([ex,ey,ez])).tolist(), vec=(l*pos).tolist())

    acts = [aux(act) for act in sol]
    basefrm = FRAME(xyzabc=[300,0,800,0,0,0])
    for a in acts:
        nm = 'piece-'+a[0]
        f = basefrm*a[1]
        print 'put ', nm, 'at ', f
        env.get_object(nm).locate(f)
        if step:
            raw_input(); print('hit any key')


setup_puzzle_scene()
r.prepare()
colored_print("demo()", 'blue')
