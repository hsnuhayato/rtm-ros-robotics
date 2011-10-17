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

def setup_puzzle_scene():
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
    '''given an object, compute handlink frame and distance between fingers to pick the piece'''
    l = 30
    d = 30

    grspposs = {
        'piecedg':FRAME(xyzabc=[0,0,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]), # dark green
        'pieceyg':FRAME(xyzabc=[0,l,0,-pi/2,0,0]), # yellow green
        'piecerd':FRAME(xyzabc=[-0.5*l,-l,0,pi/2,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]), # red
        'piecepp':FRAME(xyzabc=[l,0,0,0,pi/2,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]), # purple
        'piecelb':FRAME(xyzabc=[0,2*l,0,-pi/2,0,0])*FRAME(xyzabc=[0,0,0,0,0,-pi/2]), # light blue
        'piecebr':FRAME(xyzabc=[0.5*l,l,0,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]), # brown
        'pieceyl':FRAME(xyzabc=[l,l,0,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]) # yellow
    }

    gfrm = obj.where()*grspposs[obj.name]*(-r.Trwrist_ef)
    afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0])
    awidth = 80
    gwidth = 20
    return afrm,gfrm,awidth,gwidth


def place_plan(obj,p=FRAME(xyzabc=[0,0,0,0,0,0])):
    l = 30
    d = 30

    rfrm = env.get_object('piecedg').where()*FRAME(xyzabc=[0,0,0,0,0,pi/2])

    plcposs = {
        'piecedg':FRAME(xyzabc=[0,0,l,0,0,0]),
        'pieceyg':FRAME(xyzabc=[2*l,0,l,0,0,0]),
        'piecerd':FRAME(xyzabc=[0.5*l,-l,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]),
        'piecepp':FRAME(xyzabc=[2*l,-2*l,l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]),
        'piecelb':FRAME(xyzabc=[0,-2*l,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,-pi/2]),
        'piecebr':FRAME(xyzabc=[1.5*l,-l,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2]),
        'pieceyl':FRAME(xyzabc=[l,0,2*l,0,0,0])*FRAME(xyzabc=[0,0,0,0,0,pi/2])
    }

    gfrm = rfrm*plcposs[obj.name]*(-r.Trwrist_ef)        
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

def place_piece(obj,p=FRAME(xyzabc=[0,0,0,0,0,0]), jts='rarm', hand='right'):
    afrm,gfrm,width = place_plan(obj,p)
    q0 = r.get_joint_angles(joints=jts)
    q1 = r.ik(afrm, joints=jts)[0]
    traj = pl.make_plan(q0, q1, joints=jts)
    exec_traj(traj, joints=jts)
    
    r.set_joint_angles(r.ik(gfrm, joints=jts)[0], joints=jts)
    r.grasp(width, hand=hand)

def demo(jts='rarm'):
    hand = 'right' if re.match('.*rarm', jts) else 'left'
    
    cl = ['dg','yg','rd','pp','lb','br','yl']
    for c in cl:
        obj = env.get_object('piece'+c)
        r.grasp(80, hand=hand)
        pick_piece(obj, jts=jts, hand=hand)
        grab(hand=hand)
        place_piece(obj, jts=jts, hand=hand)
        release()
    r.prepare()


setup_puzzle_scene()
r.prepare()
colored_print("demo()", 'blue')
