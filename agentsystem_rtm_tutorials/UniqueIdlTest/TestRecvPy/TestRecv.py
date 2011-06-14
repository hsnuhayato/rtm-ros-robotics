#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

import RTC
import OpenRTM_aist
import Idltest

testrecv_spec = ["implementation_id", "TestRcv",
                 "type_name",         "TestRcv",
                 "description",       "Console input component",
                 "version",           "1.0",
                 "vendor",            "JSK",
                 "category",          "example",
                 "activity_type",     "DataFlowComponent",
                 "max_instance",      "10",
                 "language",          "Python",
                 "lang_type",         "script",
                 ""]

class TestRcv(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    idmsg = Idltest.Idmsg(0, "None")
    self.RecvMsg = Idltest.TimedIdmsg(RTC.Time(0,0), idmsg)
    self._inport = OpenRTM_aist.InPort("RcvMsg", self.RecvMsg)
    # Set InPort
    self.addInPort("RcvMsg", self._inport)

    print 'Initializing..'
    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print 'Activated.'
    return RTC.RTC_OK
        
  def onExecute(self, ec_id):
    if self._inport.isNew():
      rcvmsg = self._inport.read()
      print rcvmsg.data
    return RTC.RTC_OK

def TestRcvInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=testrecv_spec)
  manager.registerFactory(profile,
                          TestRcv,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  TestRcvInit(manager)

  # Create a component
  comp = manager.createComponent("TestRcv")

def main():
  # Initialize manager
  mgr = OpenRTM_aist.Manager.init(sys.argv)

  # Set module initialization proceduer
  # This procedure will be invoked in activateManager() function.
  mgr.setModuleInitProc(MyModuleInit)

  # Activate manager and register to naming service
  mgr.activateManager()

  # run the manager in blocking mode
  # runManager(False) is the default
  mgr.runManager()

  # If you want to run the manager in non-blocking mode, do like this
  # mgr.runManager(True)

if __name__ == "__main__":
  main()
