#!/usr/bin/env python
#for ros
import roslib; roslib.load_manifest('rtmros_msgpassing_python')
import rospy
from std_msgs.msg import String
#for rtm
import sys
import RTC
import OpenRTM_aist

#rtm cmoponent definition
rtmtalker_spec = ["implementation_id", "rtmtalker",
                  "type_name",         "rtmtalker",
                  "description",       "message talker component",
                  "version",           "1.0",
                  "vendor",            "JSK",
                  "category",          "example",
                  "activity_type",     "DataFlowComponent",
                  "max_instance",      "10",
                  "language",          "Python",
                  "lang_type",         "script",
                  ""]

globaldata = "None"

class rtmtalker(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        self._data = RTC.TimedString(RTC.Time(0,0), "None")
        self._outport = OpenRTM_aist.OutPort("out", self._data)
        # Set OutPort buffer                                                                                                                    
        self.addOutPort("out", self._outport)
        return RTC.RTC_OK

    def onExecute(self, ec_id):
        global globaldata
        self._data.data = globaldata
        OpenRTM_aist.setTimestamp(self._data)
        self._outport.write()
        return RTC.RTC_OK


def rtmtalkerInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=rtmtalker_spec)
    manager.registerFactory(profile,
                            rtmtalker,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    rtmtalkerInit(manager)
    # Create a component                                                                                                                      
    comp = manager.createComponent("rtmtalker")

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
    mgr.runManager(True)

    # If you want to run the manager in non-blocking mode, do like this                                                                       
    # mgr.runManager(True)                                                                                                                    

#node definition for ros
def callback(data):
    global globaldata
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    globaldata = data.data

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
    listener()
