# -*- coding: utf-8 -*-
# This is a copy of iv_plan/src/set_env.py.

import os
import sys
from os import path

ros_available = False

try:
    ivpkgdir = os.environ['IV_PKG_DIR']
except:
    ivpkgdir = path.abspath('../..')
    print "'IV_PKG_DIR' is not set, so use %s instead." % ivpkgdir

if ros_available:
    import roslib; roslib.load_manifest('iv_plan')
else:
    def load_manifest(pkgnm):
        sys.path.append(ivpkgdir+pkgnm+'/src')

    for pkgnm in ['/iv_plan','/iv_idl','/rtc_handle','/rmrc_geo_model']:
        load_manifest(pkgnm)

    sys.path.append(ivpkgdir+'/iv_plan/lib')
    sys.path.append(ivpkgdir+'/iv_plan/externals/visual/site-packages/')
