# -*- coding: utf-8 -*-

import os
import sys

try:
    pkgdir = os.environ['IV_PLAN_ROOT']
except:
    pkgdir = '..'

ros_available = True
if ros_available:
    import roslib; roslib.load_manifest('iv_plan')
else:
    libpath = os.environ['HOME'] + '/prog/hironx/agenssystem_hironx_samples'
    paths = [libpath+'/iv_plan',
             libpath+'/iv_idl',
             libpath+'/rtc_handle',
             libpath+'/rmrc_geo_model']
    for p in paths:
        sys.path.append(p)


sys.path.append(pkgdir+'/lib')

# for patched python visual
# shared install
# sys.path.append('/usr/local/lib/python2.6/site-packages/')
# user install
sys.path.append(pkgdir+'/externals/visual/site-packages/')
