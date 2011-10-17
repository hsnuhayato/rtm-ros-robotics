# -*- coding: utf-8 -*-

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
    return env.eval_sctree({'type':'parts', 'name':'piece'+nametag, 'shape':None, 'children':cbs})

init_frames = {}

def init_puzzle_scene():
    l = 30.0
    tbl = env.get_object('table')
    tbltop = env.get_object('table top')
    ttz = tbltop.where().vec[2] + tbltop.vbody.size[2]/2
    z = ttz + l/2

    # generate 7 pieces
    # dark green piece
    env.insert_object(generate_piece('dg', (0.0, 0.392, 0.0),
                                     [(0,0,0,0,0,0),(0,0,l,0,0,0),(l,0,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-300,-50,z,0,0,-pi/2]), tbl)
    # yellow green piece
    env.insert_object(generate_piece('yg', (0.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(-l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-180,0,z,pi/2,0,0]), tbl)
    # red piece
    env.insert_object(generate_piece('rd', (1.0, 0.0, 0.0),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(0,-l,0,0,0,0),(-l,-l,0,0,0,0)]),
                      FRAME(xyzabc=[-300,-200,z,-pi/2,0,0]), tbl)
    # purple piece
    env.insert_object(generate_piece('pp', (0.6, 0.196, 0.8),
                                     [(0,0,0,0,0,0),(0,0,l,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-180,-150,z,0,-pi/2,0]), tbl)
    # light blue piece
    env.insert_object(generate_piece('lb', (0.0, 1.0, 1.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(0,2*l,0,0,0,0),(l,l,0,0,0,0)]),
                      FRAME(xyzabc=[-350,-300,z,pi/2,0,0]), tbl)
    # brown piece
    env.insert_object(generate_piece('br', (0.545, 0.27, 0.075),
                                     [(0,0,0,0,0,0),(l,0,0,0,0,0),(l,l,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-250,-300,z,0,0,0]), tbl)
    # yellow piece
    env.insert_object(generate_piece('yl', (1.0, 1.0, 0.0),
                                     [(0,0,0,0,0,0),(0,l,0,0,0,0),(l,l,0,0,0,0),(l*2,l,0,0,0,0)]),
                      FRAME(xyzabc=[-350,0,z,0,0,0]), tbl)

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
    ''' given an object, compute handlink pose and distance between fingers '''
    l = 30.0
    d = 30.0
    # target frame needs default coordinate
    if obj.name == 'piecedg':
        # dark green
        Tobj_grsp = FRAME(xyzabc=[0,0,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    elif obj.name == 'pieceyg':
        # yellow green
        Tobj_grsp = FRAME(xyzabc=[0,l,0,-pi/2,0,0])
    elif obj.name == 'piecerd':
        # red
        Tobj_grsp = FRAME(xyzabc=[-0.5*l,-l,0,pi/2,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    elif obj.name == 'piecepp':
        # purple
        Tobj_grsp = FRAME(xyzabc=[l,0,0,0,pi/2,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    elif obj.name == 'piecelb':
        # light blue
        Tobj_grsp = FRAME(xyzabc=[0,2*l,0,-pi/2,0,0])*FRAME(xyzabc=[0,0,0,0,0,-pi/2])
    elif obj.name == 'piecebr':
        # brown
        Tobj_grsp = FRAME(xyzabc=[0.5*l,l,0,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    elif obj.name == 'pieceyl':
        # yellow
        Tobj_grsp = FRAME(xyzabc=[l,l,0,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    else:
        return warn('unknown piece')

    gfrm = obj.where()*Tobj_grsp*(-r.Trwrist_ef)
    afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
    return afrm,gfrm


def place_plan(obj,p=FRAME(xyzabc=[0,0,0,0,0,0])):
    l = 30
    d = 30
    rfrm = env.get_object('piecedg').where()*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    if obj.name == 'piecedg':
        gfrm = rfrm*FRAME(xyzabc=[0,0,l,0,0,0])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'pieceyg':
        gfrm = rfrm*FRAME(xyzabc=[2*l,0,l,0,0,0])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'piecerd':
        gfrm = rfrm*FRAME(xyzabc=[0.5*l,-l,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'piecepp':
        gfrm = rfrm*FRAME(xyzabc=[2*l,-2*l,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'piecelb':
        gfrm = rfrm*FRAME(xyzabc=[0,-2*l,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,-pi/2])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'piecebr':
        gfrm = rfrm*FRAME(xyzabc=[1.5*l,-l,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    elif obj.name == 'pieceyl':
        gfrm = rfrm*FRAME(xyzabc=[l,0,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
        return afrm,gfrm
    else:
        warn('unknown piece')

def pick_piece(obj):
    afrm,gfrm = grasp_plan(obj)
    jts = 'rarm'
    r.set_joint_angles(r.ik(afrm, joints=jts)[0], joints=jts)
    raw_input()
    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    return afrm,gfrm

def place_piece(obj,p=FRAME(xyzabc=[0,0,0,0,0,0])):
    afrm,gfrm = place_plan(obj,p)
    jts = 'rarm'
    r.set_joint_angles(r.ik(afrm, joints=jts)[0], joints=jts)
    raw_input()
    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    return afrm,gfrm

def demo():
    cl = ['dg','yg','rd','pp','lb','br','yl']
    for c in cl:
        obj = env.get_object('piece'+c)
        r.grasp(80,hand='right')
        pick_piece(obj)
        raw_input()
        r.grasp(20,hand='right')
        grab(hand='right')
        place_piece(obj)
        raw_input()
        release()
    r.prepare()

init_puzzle_scene()
r.prepare()
