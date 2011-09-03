# -*- coding: utf-8 -*-

import os
import sys
from os import path

real_robot = False
ros_available = False

ivpkgdir = os.path.abspath('../..')
ivpkgs = ['/iv_plan','/iv_idl','/rtc_handle','/rmrc_geo_model']

if ros_available:
    import roslib; roslib.load_manifest('iv_plan')
    import rospy
    #nameserver = rospy.get_param('hiro/nameserver')

else:
    def load_manifest(pkgnm):
        for subdir in ['src', 'lib']:
            sys.path.append(ivpkgdir+pkgnm+'/'+subdir)

    for pkg in ivpkgs:
        load_manifest(pkg)

sys.path.append(ivpkgdir+'/iv_plan/externals/visual/site-packages/')

def getNameServerFromConf(argv):
    import OpenRTM_aist
    mc = OpenRTM_aist.ManagerConfig(argv)
    prop = OpenRTM_aist.Properties()
    mc.configure(prop)
    nameserver = prop.findNode('corba.nameservers').getValue()
    print 'config file = %s' % mc._configFile
    print 'nameserver = %s' % nameserver
    return nameserver

nameserver = getNameServerFromConf(sys.argv)

# if real_robot:
#     nameserver = 'hiro014'
# else:
#     nameserver = 'localhost'
