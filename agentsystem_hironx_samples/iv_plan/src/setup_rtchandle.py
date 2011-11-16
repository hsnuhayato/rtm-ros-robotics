# -*- coding: utf-8 -*-

from set_env import *
from ivutils import *
from rtc_handle import *
import RTC
import Img
#import OpenHRP
import _GlobalIDL


try:
    rtmenv = rtc_handle.RtmEnv(sys.argv, [nameserver])
    ns = rtmenv.name_space[nameserver]
    ns.list_obj()
except:
    warn('HIRO-NX base system is not running')
    warn('or rtc.conf is not correctly configured')

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


# global h_seq, seq_svc, jstt_port, mysvc, h_my
# h_seq = get_handle('seq.rtc', ns)
# seq_svc = narrow_service(h_seq, 'SequencePlayerService', 'service0')
# h_my = get_handle('MyServiceProvider0.rtc', ns)
# mysvc = narrow_service(h_my, 'MyService', 'myservice0')
