#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('iv_plan')
from set_env import *
# from demo_common import *
# import time
# import random
# from utils import *
import visual
# from viewer import *



import RTC
import _GlobalIDL, _GlobalIDL__POA
import OpenRTM_aist


# Module specification
mplanserviceprovider_spec = ["implementation_id", "MPlan",
                             "type_name",         "MPlan",
                             "description",       "Planner for HIRO-NX",
                             "version",           "0.1.0",
                             "vendor",            "Ryo Hanai",
                             "category",          "Planner",
                             "activity_type",     "EVENTDRIVEN",
                             "kind",              "DataFlowComponent",
                             "max_instance",      "0",
                             "language",          "Python",
                             "lang_type",         "script",
                             ""]


class MPlanServiceSVC_impl(_GlobalIDL__POA.ArmMotionService):
    def __init__(self):
        return
    
    def __del__(self):
        pass

class MPlanServiceProvider(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        self._armPlanServicePort = OpenRTM_aist.CorbaPort("MPlanService")
        self._service0 = MPlanServiceSVC_impl()
        self._armPlanServicePort.registerProvider("service0",
                                                  "MPlanService",
                                                  self._service0)
        self.addPort(self._armPlanServicePort)

        return RTC.RTC_OK

def MPlanInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=mplanserviceprovider_spec)
    manager.registerFactory(profile,
                            MPlanServiceProvider,
                            OpenRTM_aist.Delete)
    return


def MPlanModuleInit(manager):
    MPlanInit(manager)
    comp = manager.createComponent("MPlan")
    return


def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)

    mgr.setModuleInitProc(MPlanModuleInit)

    mgr.activateManager()

    mgr.runManager()


if __name__ == "__main__":
    main()
