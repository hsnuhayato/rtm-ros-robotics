# -*- coding: utf-8 -*-

import os
import sys
from os import path

ros_available = False
real_robot = False

try:
    ivpkgdir = os.environ['IV_PKG_DIR']
except:
    ivpkgdir = path.abspath('../..')
    print "'IV_PKG_DIR' is not set, so use %s instead." % ivpkgdir


if ros_available:
    import roslib; roslib.load_manifest('iv_plan')
    import rospy
    nameserver = rospy.get_param('hiro/nameserver')

else:
    def load_manifest(pkgnm):
        sys.path.append(ivpkgdir+pkgnm+'/src')

    for pkgnm in ['/iv_plan','/iv_idl','/rtc_handle','/rmrc_geo_model']:
        load_manifest(pkgnm)

    sys.path.append(ivpkgdir+'/iv_plan/lib')
    sys.path.append(ivpkgdir+'/iv_plan/externals/visual/site-packages/')

    if real_robot:
        nameserver = 'hiro014'
    else:
        nameserver = 'localhost'
