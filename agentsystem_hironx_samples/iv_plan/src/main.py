# -*- coding: utf-8 -*-

from utils import *
from viewer import *
import scene_objects
from robot import *
from mplan_env import *
from csplan import *


real_robot = False
if real_robot:
    from real_hiro import *
    import rospy
    rr = RealHIRO()
else:
    rr = None

env = MPlanEnv()
env.load_scene(scene_objects.ac_scene())
r = VHIRONX(ivpkgdir+'/iv_plan/externals/models/HIRONX_110822/')
env.insert_robot(r)
r.go_pos(-150, 0, 0)
pl = CSPlanner(env)
