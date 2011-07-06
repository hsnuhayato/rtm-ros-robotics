#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

# standard IDL installation path
# /usr/lib/python2.6/dist-packages/OpenRTM_aist/RTM_IDL/RTC$ 


import sys
import time

import RTC
import OpenRTM_aist

import roslib; roslib.load_manifest('iv_bridges')
import rospy
from sensor_msgs.msg import JointState

import operator

consoleout_spec = ["implementation_id", "JointStatePublisher",
                   "type_name",         "JointStatePublisher",
                   "description",       "JointStatePublisher",
                   "version",           "1.0",
                   "vendor",            "Ryo Hanai",
                   "category",          "example",
                   "activity_type",     "DataFlowComponent",
                   "max_instance",      "10",
                   "language",          "Python",
                   "lang_type",         "script",
                   ""]


class DataListener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self, name):
    self._name = name

  def __del__(self):
    print "dtor of ", self._name

  def __call__(self, info, cdrdata):
    data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedJointData(RTC.Time(0,0),[],[],[],[],[],[],[],[]))

class JointStatePublisher(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    self.js = JointState()
    self.js.name = ["chest","head0","head1", 
                    "rarm0","rarm1","rarm2","rarm3","rarm4","rarm5",
                    "larm0","larm1","larm2","larm3","larm4","larm5",
                    "rhand0","rhand1","rhand2","rhand3",
                    "lhand0","lhand1","lhand2","lhand3"]
    self.js.header.frame_id = 'hiro'

    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    self._data = RTC.TimedJointData(RTC.Time(0,0),[],[],[],[],[],[],[],[])
    self._inport = OpenRTM_aist.InPort("in", self._data)
    # Set InPort buffer
    self.addInPort("in", self._inport)

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                                          DataListener("ON_BUFFER_WRITE"))


    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL, 
                                          DataListener("ON_BUFFER_FULL"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE_TIMEOUT, 
                                          DataListener("ON_BUFFER_WRITE_TIMEOUT"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_OVERWRITE, 
                                          DataListener("ON_BUFFER_OVERWRITE"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ, 
                                          DataListener("ON_BUFFER_READ"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_SEND, 
                                          DataListener("ON_SEND"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED,
                                          DataListener("ON_RECEIVED"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL, 
                                          DataListener("ON_RECEIVER_FULL"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_TIMEOUT, 
                                          DataListener("ON_RECEIVER_TIMEOUT"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_ERROR,
                                          DataListener("ON_RECEIVER_ERROR"))

    return RTC.RTC_OK

  def onExecute(self, ec_id):

    if self._inport.isNew():
      data = self._inport.read()
      self.js.header.stamp.secs = data.tm.sec
      self.js.header.stamp.nsecs = data.tm.nsec
      self.js.position = reduce(operator.__concat__, data.qState)
      self.js.velocity = reduce(operator.__concat__, data.dqState)
      jspub.publish(self.js)

    return RTC.RTC_OK


def JointStatePublisherInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=consoleout_spec)
  manager.registerFactory(profile,
                          JointStatePublisher,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  JointStatePublisherInit(manager)

  # Create a component
  comp = manager.createComponent("JointStatePublisher")


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
  global jspub
  jspub = rospy.Publisher('joint_state', JointState)
  rospy.init_node('joint_state_publisher')

  main()
