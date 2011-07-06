##
## scenario.py
## R.Hanai 2011.05.23 -
##

import roslib; roslib.load_manifest('iv_plan')

from rtc_handle import *
import RTC
import _GlobalIDL
import rospy
import math

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

nameserver = 'localhost:2809'
env = RtmEnv(sys.argv, [nameserver])
ns = env.name_space[nameserver]
ns.list_obj()

hpl = get_handle('kiwi2.host_cxt/MPlan0.rtc', ns)
hpl.activate()
plsvc = hpl.services['MPlan0.ArmPlanService'].provided['service0']

# armplan_svc = narrow_service(h_armplan, 'MotionPlan0.MotionPlanService', 'service0')

# Data type definitions:
#  /usr/include/rtm/idl/ExtendedDataTypes.idl
def gen_goal_frm(x=180,y=-175,z=900,a=0,b=-math.pi/2.0,c=0):
    pt = RTC.Point3D(x, y, z)
    ori = RTC.Orientation3D(a, b, c)
    frm = RTC.Pose3D(pt, ori)
    return frm

# is_success = plsvc.ref.MoveArm(frm, 100, 'right', False, False)

