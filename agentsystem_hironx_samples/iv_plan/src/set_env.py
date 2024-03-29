# -*- coding: utf-8 -*-

import os
import sys
from os import path

real_robot = False
ros_available = True

#ivpkgdir = os.path.abspath('../..')
#ivpkgdir = os.environ['IV_PKG_DIR']
ivpkgdir = os.path.dirname(os.path.abspath(__file__))+'/../..'
print 'IVPKGDIR2:', ivpkgdir
ivpkgs = ['/iv_plan','/iv_idl','/rtc_handle','/rmrc_geo_model','/tfpy']

if ros_available:
    import roslib; roslib.load_manifest('iv_plan')
    import rospy
    #nameserver = rospy.get_param('hiro/nameserver')

else:
    def load_manifest(pkgnm):
        for subdir in ['src', 'lib']:
            sys.path.insert(0, ivpkgdir+pkgnm+'/'+subdir)

    for pkg in ivpkgs:
        load_manifest(pkg)

sys.path.insert(0, ivpkgdir+'/iv_plan/externals/visual/site-packages/')

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
