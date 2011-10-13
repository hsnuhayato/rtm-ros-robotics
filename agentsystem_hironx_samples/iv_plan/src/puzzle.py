# -*- coding: utf-8 -*-

from numpy import *
import sys
import time

from demo_common import *

# definitions of piece geometry
def cube(name, color, l):
    return {'name':name, 'shape':'box', 'color':color, 'material':None,
            'dimension':[l, l, l], 'children':[]}

def generate_piece(nametag, color, cubeposes, l=30.0):
    l2 = l/2
    xyzabcs = cubeposes
    cbs = [(cube(name=nametag+str(i), color=color, l=l),xyzabc) for i,xyzabc in enumerate(xyzabcs)]
    return env.eval_sctree({'name':'piece'+nametag, 'shape':None, 'children':cbs})

def init_puzzle_scene():
    l = 30.0
    tbl = env.get_object('table')

    # generate 7 pieces
    env.insert_object(generate_piece('yg', (0.0, 1.0, 1.0), [(0,0,0,0,0,0),(l,0,0,0,0,0),(0,l,0,0,0,0)]),
                      FRAME(xyzabc=[-260,-10,714,0,0,pi/6]), tbl)

    # define collision pairs

def grasp_plan(obj):
    # given object, compute handlink pose and distance between fingers
    l = 30.0
    d = 30.0
    if obj.name == 'pieceyg':
        gfrm = obj.where()*FRAME(xyzabc=[l,0,0,0,0,0])*(-r.Trwrist_ef)
        afrm = gfrm*FRAME(xyzabc=[d,0,0,0,0,0,0])
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


init_puzzle_scene()
obj = env.get_object('pieceyg')
