#!/usr/bin/env python
import sys
import RTC
import OpenRTM_aist

#rtm cmoponent definition
rtmlistener_spec = ["implementation_id", "rtmlistener",
                    "type_name",         "rtmlistener",
                    "description",       "message listener component",
                    "version",           "1.0",
                    "vendor",            "JSK",
                    "category",          "example",
                    "activity_type",     "DataFlowComponent",
                    "max_instance",      "10",
                    "language",          "Python",
                    "lang_type",         "script",
                    ""]

class rtmlistener(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        return

    def onInitialize(self):
        self._data = RTC.TimedString(RTC.Time(0,0), "None")
        self._inport = OpenRTM_aist.InPort("in", self._data)
        # Set InPort buffer                                                                                                                    
        self.addInPort("out", self._inport)
        return RTC.RTC_OK

    def onExecute(self, ec_id):
        if self._inport.isNew():
            data = self._inport.read()
            print data.data

        return RTC.RTC_OK

def rtmlistenerInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=rtmlistener_spec)
    manager.registerFactory(profile,
                            rtmlistener,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    rtmlistenerInit(manager)
    # Create a component                                                                                                                      
    comp = manager.createComponent("rtmlistener")

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

if __name__ == '__main__':
    main()

