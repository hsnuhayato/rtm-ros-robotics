##
## rtc_handle_aux.py
##
## R.Hanai 2011.06.14 -
##


## ROS
import roslib; roslib.load_manifest('iv_plan')
import rospy
##

## RTM
from rtc_handle import *
import RTC
import _GlobalIDL

try:
    nameserver = rospy.get_param('hiro/nameserver')
    env = RtmEnv(sys.argv, [nameserver])
    ns = env.name_space[nameserver]
    ns.list_obj()
except:
    warn('HIRO-NX base system is not running, or ROS_MASTER_URI is not set correctly, maybe ...')
##


def get_handle(name, nspace):
    return nspace.rtc_handles[name]

def narrow_service(handle, service, port):
    svc = handle.services[service].provided[port]
    svc.narrow_ref(globals())
    return svc

def activate(handles):
    for hdl in handles:
        hdl.activate()

def deactivate(handles):
    for hdl in handles:
        hdl.deactivate()

def connect(handle1, port1, handle2, port2):
    con = IOConnector([handle1.outports[port1], handle2.inports[port2]])
    con.connect()

def disconnect(con):
    con.disconnect()

