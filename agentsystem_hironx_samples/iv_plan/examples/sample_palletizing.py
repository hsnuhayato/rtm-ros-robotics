# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
from demo_common import *

# parts: A0,A1,A2,A3
# places: P0,P1,P2,P3

# task sequence
# A0 => P0, A1 => P1, A2 => P2, A3 => P3

def demo():
    for i in range(4):
        # if i % 2 == 0:
        #     l_or_r = 'right'
        # else:
        #     l_or_r = 'left'
        l_or_r = 'right'
        parts = env.get_object(name='A'+str(i))
        move_arm_ef(parts.where(), arm=l_or_r)
        grasp(width=parts.vbody.size[1], name=parts.name, hand=l_or_r)
        place = env.get_object(name='P'+str(i))
        f = place.where()
        f.vec[2] += 25
        move_arm_ef(f, arm=l_or_r)
        release(width=parts.vbody.size[1]+10, name=parts.name, hand=l_or_r)


def reset_world():
    for i in range(4):
        env.delete_object('A'+str(i))
    env.delete_object('table')
    env.delete_object('floor')
    env.delete_object('hirobase')
    env.load_scene(scene_objects.ac_scene())

colored_print('1: demo()', 'blue')
