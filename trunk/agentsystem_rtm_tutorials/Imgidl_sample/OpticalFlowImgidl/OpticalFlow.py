#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-
import sys

import RTC
import OpenRTM_aist
import Img
from opencv.cv import *
from opencv.highgui import *
from omniORB import CORBA

testrecv_spec = ["implementation_id", "OpticalFlow",
                 "type_name",         "OpticalFlow",
                 "description",       "Console input component",
                 "version",           "1.0",
                 "vendor",            "JSK",
                 "category",          "example",
                 "activity_type",     "DataFlowComponent",
                 "max_instance",      "10",
                 "language",          "Python",
                 "lang_type",         "script",
                 "conf.default.track_frame_flag", "0",
                 ""]

class take_one_frame_functor:
  def __init__(self):
    return

  def __call__(self, obj):
    try:
      if CORBA.is_nil(obj):
        print "No service connected."
      else:
        obj.take_one_frame()
    except:
      pass

class OpticalFlow(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    self.window_name = "OpticalFlow"
    self.track_frame_flag = [0]
    self.previmage = None
    return

  def onInitialize(self):
    # Setting Corba service port
    self._CameraCaptureServicePort = OpenRTM_aist.CorbaPort("CameraCaptureService")
    self._CameraCaptureService0 = OpenRTM_aist.CorbaConsumer(interfaceType=Img.CameraCaptureService)
    self._CameraCaptureServicePort.registerConsumer("CameraCaptureService0", "CameraCaptureService", self._CameraCaptureService0)
    self.addPort(self._CameraCaptureServicePort)

    # Setting InPort
    # temporary value for creating initial value
    width = 320
    height = 240
    channels = 3
    rdata = []
    for i in range(width*height*channels):
      rdata.append(0)

    matelem = []
    for i in range(5):
      matelem.append(0.0)

    distcoef = []
    for i in range(5):
      distcoef.append(0.0)

    intrinsicParam = Img.CameraIntrinsicParameter(matelem, distcoef)

    extrinsicParam = []
    extrinsicParam.append([1, 0, 0, 0])
    extrinsicParam.append([0, 1, 0, 0])
    extrinsicParam.append([0, 0, 1, 0])
    extrinsicParam.append([0, 0, 0, 1])

    imgdat = Img.ImageData(width, height, Img.CF_RGB, rdata)

    cimg = Img.CameraImage(RTC.Time(0,0), imgdat, intrinsicParam, extrinsicParam)
    error_code = 0L
    self._data = Img.TimedCameraImage(RTC.Time(0,0), cimg, error_code)
    self._inport = OpenRTM_aist.InPort("Image", self._data)

    self.addInPort("Image", self._inport)
    self.bindParameter("track_frame_flag", self.track_frame_flag, "0")

    print 'Initializing..'
    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print 'Activated.'
    cvNamedWindow(self.window_name, CV_WINDOW_AUTOSIZE)
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    print 'Deactivated.'
    cvDestroyWindow(self.window_name)
    return RTC.RTC_OK
        
  def onExecute(self, ec_id):
    if self.track_frame_flag[0]:
      func = take_one_frame_functor()
      self._async_takeframe = OpenRTM_aist.Async_tInvoker(self._CameraCaptureService0._ptr(), func)
      self._async_takeframe.invoke()

    if self._inport.isNew():
      curimg = None
      tmcameraimg = self._inport.read()
      cameraimg = tmcameraimg.data 
      # copy TimedCameraImage data from Data Port
      width = cameraimg.image.width
      height = cameraimg.image.height
      format = cameraimg.image.format
      if format == Img.CF_RGB:
        tmpimg = cvCreateImage((width, height), IPL_DEPTH_8U, 3)
        showimg = cvCreateImage((width, height), IPL_DEPTH_8U, 3)
        tmpimg.imageData = cameraimg.image.raw_data
        cvCvtColor(tmpimg, showimg, CV_RGB2BGR)
        smallimg = cvCreateImage((width/2, height/2), IPL_DEPTH_8U, 3)
        cvPyrDown(showimg, smallimg,  CV_GAUSSIAN_5x5)
        curimg = cvCreateImage((width/2, height/2), IPL_DEPTH_8U, 1)
        cvCvtColor(smallimg, curimg, CV_RGB2GRAY)
      else:
        print 'unimplemented format'
        return RTC.RTC_OK

      if self.previmage != None:
        ob0 = cvCreateImage((width/2, height/2), IPL_DEPTH_32F, 1)
        ob1 = cvCreateImage((width/2, height/2), IPL_DEPTH_32F, 1)
        cvCalcOpticalFlowLK(self.previmage, curimg, (11, 11), ob0, ob1) #領域(u,v)サイズ=(15,15)
        xp = []
        yp = []
        for i in range(0, height/2, 10):
          for j in range(0, width/2, 10):
            if not (cvGet2D(ob0,i,j)[0] == 0 and cvGet2D(ob1,i,j)[0]==0):
              xp.append([i, j, cvGet2D(ob0, i, j)[0]])
              yp.append([i, j, cvGet2D(ob1, i, j)[0]])

        for k in range(0,len(xp)):
          cvLine(smallimg, (xp[k][1], xp[k][0]), (xp[k][1]+xp[k][2], xp[k][0]+yp[k][2]), (0,0,255), 1, CV_AA, 0)

      cvShowImage(self.window_name, smallimg)
      cvWaitKey(2)
      self.previmage = curimg

    return RTC.RTC_OK

def OpticalFlowInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=testrecv_spec)
  manager.registerFactory(profile,
                          OpticalFlow,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  OpticalFlowInit(manager)

  # Create a component
  comp = manager.createComponent("OpticalFlow")

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
