#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

import RTC
import OpenRTM_aist
import Img, Img__POA
from opencv.cv import *
from opencv.highgui import *

testrecv_spec = ["implementation_id", "UsbCameraCapture",
                 "type_name",         "UsbCameraCapture",
                 "description",       "Console input component",
                 "version",           "1.0",
                 "vendor",            "JSK",
                 "category",          "example",
                 "activity_type",     "DataFlowComponent",
                 "max_instance",      "10",
                 "language",          "Python",
                 "lang_type",         "script",
                 ""]


class ImgServiceSVC_impl(Img__POA.CameraCaptureService):
  def __init__(self):
    self._cap_count = 0
    self._cap_continuous = False
    return

  def __del__(self):
    pass

  def take_one_frame(self):
    self._cap_count = 1

  def take_multi_frames(self, num):
    self._cap_count = num

  def start_continuous(self):
    self._cap_continuous = True

  def stop_continuous(self):
    self._cap_continuous = False

class UsbCameraCapture(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    self.window_name = "Captured Image"
    self.cap = None
    self.width = 640
    self.height = 480
    self.channels = 3
    self.camera_id = [0]
    self.output_color_format = ["RGB"]
    self.camera_param_filename = ["camera.yml"]
    self.undistortion_flag = [0]
    return

  def onInitialize(self):
    # Setting Serviceport
    self._CameraCaptureServicePort = OpenRTM_aist.CorbaPort("CameraCaptureService")
    self._CameraCaptureService0 = ImgServiceSVC_impl()
    self._CameraCaptureServicePort.registerProvider("CameraCaptureService0", "CameraCaptureService", self._CameraCaptureService0)
    self.addPort(self._CameraCaptureServicePort)

    # Setting OutPort    
    rdata = []
    for i in range(self.width*self.height*self.channels):
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

    imgdat = Img.ImageData(self.width, self.height, Img.CF_RGB, rdata)
    cimg = Img.CameraImage(RTC.Time(0,0), imgdat, intrinsicParam, extrinsicParam)
    error_code = 0L
    self._data = Img.TimedCameraImage(RTC.Time(0,0), cimg, error_code)
    self._outport = OpenRTM_aist.OutPort("Image", self._data)

    self.addOutPort("Image", self._outport)
    self.bindParameter("camera_id", self.camera_id, "0")
    self.bindParameter("output_color_format", self.output_color_format, "RGB")
    self.bindParameter("camera_param_filename", self.camera_param_filename, "camera.yml")
    self.bindParameter("undistortion_flag", self.undistortion_flag, "0")
    print 'Initializing..'
    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print 'Activated.'
    self.cap = cvCreateCameraCapture(self.camera_id[0])
    cvSetCaptureProperty(self.cap, CV_CAP_PROP_FRAME_WIDTH, self.width)
    cvSetCaptureProperty(self.cap, CV_CAP_PROP_FRAME_HEIGHT, self.height)
    cvNamedWindow(self.window_name, CV_WINDOW_AUTOSIZE)
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    print 'Deactivated.'
    cvReleaseCapture(self.cap)
    cvDestroyWindow(self.window_name)
    return RTC.RTC_OK
        
  def onExecute(self, ec_id):
    if self._CameraCaptureService0._cap_continuous:
      pass
    elif self._CameraCaptureService0._cap_count > 0:
      self._CameraCaptureService0._cap_count-=1
    else:
      print "Waiting capture command from continuous_flag or Service Port."
      return RTC.RTC_OK

    frame = cvQueryFrame(self.cap)

    #set data
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

    imgdat = Img.ImageData(self.width, self.height, Img.CF_RGB, frame.imageData)
    cimg = Img.CameraImage(RTC.Time(0,0), imgdat, intrinsicParam, extrinsicParam)

    self._data.data = cimg
    OpenRTM_aist.setTimestamp(self._data)
    self._outport.write()

    return RTC.RTC_OK

def UsbCameraCaptureInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=testrecv_spec)
  manager.registerFactory(profile,
                          UsbCameraCapture,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  UsbCameraCaptureInit(manager)

  # Create a component
  comp = manager.createComponent("UsbCameraCapture")

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
