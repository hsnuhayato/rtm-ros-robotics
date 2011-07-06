#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
from robot import *
from csplan import *

class aclass:
    def __init__(self):
        pass

    def move_arm(goalori, goalpos, width):
        """
        This is the explanation of the function, hoge.
        @rtc_interface: unsigned long MoveArm(in Mat33, in Vec3, in double)
        """
        print 'move_arm'

    def aho():
        """
        abc def
        @rtc_interface: unsigned int aho()
        ghi
        """
        pass
    
    def fuga():
        print "fuga has no documentation"


def gen_doc(clsname, instname):
    obj = eval(clsname)
    attrnames = dir(obj)

    instclass = aclass.__init__.__class__
    mthds = []
    for attrname in attrnames:
        attr = eval(clsname + '.' + attrname)
        if attr.__class__ == instclass:
            if attr.__doc__:
                # print attr.__doc__
                ifdesc = re.search('@rtc_interface: [^\n]*', attr.__doc__)
                if ifdesc:
                    sig = re.sub('@rtc_service_signature: ', '', ifdesc.group())
                    args, valtyp = re.split('->', sig)
                    arg_types = re.split(',', re.sub('(.*\()|(\).*)', '', args))
                    arg_names = attr.func_code.co_varnames
                    py_def = ('def ' + idl_def + '\n'
                              + '\treturn ' + instname + '.' + attrname + '()')
                    mthds.append((idl_def, py_def))
    return mthds

idl_header = '''
interface MotionPlanService
{
    typedef sequence<double, 3> Vec3;
    typedef sequence<sequence<double, 3>, 3> Mat33;
    typedef sequence<double> DoubleSequence;

    // Make an arm plan and execute the planned motion
'''

idl_footer = '''
    typedef sequence<sequence<double> > Trajectory;

    struct Frame {
        Mat33 mat;
        Vec3 vec;
    };

    typedef sequence<Frame> FrameSequence;
};
'''

comp_header = '''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time


## ros dependent
import roslib; roslib.load_manifest('MotionPlan')
##


import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist

from mysample import *

c_frame = _GlobalIDL.MotionPlanService.Frame

# Module specification
motionplanserviceprovider_spec = ["implementation_id", "MotionPlan",
                                  "type_name",         "MotionPlan",
                                  "description",       "Motion Planner",
                                  "version",           "0.1.0",
                                  "vendor",            "Ryo Hanai",
                                  "category",          "Planner",
                                  "activity_type",     "EVENTDRIVEN",
                                  "kind",              "DataFlowComponent",
                                  "max_instance",      "0",
                                  "language",          "Python",
                                  "lang_type",         "script",
                                  ""]


class MotionPlanServiceSVC_impl(_GlobalIDL__POA.MotionPlanService):
    def __init__(self):
        return
    
    def __del__(self):
        pass

'''

comp_footer = '''
class MotionPlanServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        # initialization of CORBA Port
        self._motionPlanServicePort = OpenRTM_aist.CorbaPort("MotionPlanService")

        # initialization of Provider
        self._service0 = MotionPlanServiceSVC_impl()

        # Set service providers to Ports
        self._motionPlanServicePort.registerProvider("service0",
                                                  "MotionPlanService",
                                                  self._service0)

        # Set CORBA Service Ports
        self.addPort(self._motionPlanServicePort)

        return RTC.RTC_OK


def MotionPlanInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=motionplanserviceprovider_spec)
    manager.registerFactory(profile,
                            MotionPlanServiceProvider,
                            OpenRTM_aist.Delete)
    return


def MotionPlanModuleInit(manager):
    MotionPlanInit(manager)
    # Create a component
    comp = manager.createComponent("MotionPlan")
    return


def main():
    # Initialize manager
    mgr = OpenRTM_aist.Manager.init(sys.argv)

    # Set module initialization proceduer
    # This procedure will be invoked in activateManager() function.
    mgr.setModuleInitProc(MotionPlanModuleInit)

    # Activate manager and register to naming service
    mgr.activateManager()

    # run the manager in blocking mode
    # runManager(False) is the default
    mgr.runManager()

    # If you want to run the manager in non-blocking mode, do like this
    # mgr.runManager(True)


if __name__ == "__main__":
    main()
'''

def generate():
    print idl_header
    for clsnm, instnm in [('CSPlanner','pl'),('VHIRONX','r')]:
        mthds = gen_doc(clsnm, instnm)
        for mthd in mthds:
            print '    '+mthd[0]+';'
            print '    '+mthd[1]
    print idl_footer

    # print comp_header
    # print comp_footer
