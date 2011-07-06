#!/opt/grx/bin/hrpsyspy
import os
import sys
import socket
import math
import time
import rtm
import waitInput
import bodyinfo
import java.lang.System
import org.omg.CORBA.DoubleHolder
import OpenHRP
from OpenHRP.RobotHardwareServicePackage import SwitchStatus

import pickle

def start_service(host = 'localhost', port = 10103):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host,port))
    s.listen(1)
    print 'Waiting for connections...'

    while True:
        clientsock, clientaddr = s.accept()
        try:
            msgstr = clientsock.recv(4096)
            msg = pickle.loads(msgstr)
            print 'Received: ', msg
            pose, duration, wait = msg
            if not wait:
              msg = 'started'
              clientsock.sendall(pickle.dumps(msg))
            setJointAngles(pose, duration, wait=wait)
            if wait:
              msg = 'completed'
              clientsock.sendall(pickle.dumps(msg))
        except EOFError:
            print 'Connection closed'
            clientsock.close()
            # break

def setJointAngles(pose, tm, ttm = org.omg.CORBA.DoubleHolder(), wait=True):
    ret = seq_svc.setJointAngles(pose, tm, ttm)
    mask = 31
    # print dir(ttm)
    print 'time=',
    print time.time()/1e8
    print 'requested duration=',
    print tm
    print 'actual duration=',
    print ttm.value
    print 'wait=',
    print wait
    print 'current status=',
    print ret
    # seq_svc.isEmpty(mask, wait)
    if wait:
      seq_svc.isEmpty(mask, wait)
    return ret

def init(robotHost=None):
    if robotHost != None:
      print 'robot host = '+robotHost
      # java.lang.System.setProperty('NS_OPT',
      #     '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
      rtm.initCORBA()

    print "creating components"
    rtcList = createComps(robotHost)

    print "connecting components"
    connectComps()

    print "activating components"
    activateComps(rtcList)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        r.start()

def initRTC(module, name):
    ms.load(module)
    return ms.create(module, name)

def setJointAnglesDeg(pose, tm, ttm = org.omg.CORBA.DoubleHolder(), wait=True):
    ret = seq_svc.setJointAngles(bodyinfo.deg2radPose(pose), tm, ttm)
    mask = 31
    if wait:
        seq_svc.isEmpty(mask, True)
    return ret

def goInitial(tm=bodyinfo.timeToInitialPose):
    setJointAnglesDeg(bodyinfo.initialPose, tm)

def goOffPose(tm=bodyinfo.timeToOffPose):
    setJointAnglesDeg(bodyinfo.offPose, tm)

def servoOn(part = 'all'):
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo ON "+part)
    if rh_svc != None:
        rh_svc.servo(part, SwitchStatus.SWITCH_ON)
        if part == 'all':
            rh_svc.servo('rhand', SwitchStatus.SWITCH_ON)
            rh_svc.servo('lhand', SwitchStatus.SWITCH_ON)

def servoOff(part = 'all'):
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo OFF "+part)
    if rh_svc != None:
        rh_svc.servo(part, SwitchStatus.SWITCH_OFF)
        if part == 'all':
            rh_svc.servo('rhand', SwitchStatus.SWITCH_OFF)
            rh_svc.servo('lhand', SwitchStatus.SWITCH_OFF)

def loadPattern(basename, tm=3.0):
    seq_svc.loadPattern(basename, tm)

def testPattern():
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to execute "+bodyinfo.testPatternName)
    dblHolder = org.omg.CORBA.DoubleHolder()
    for p in bodyinfo.testPattern:
      print setJointAnglesDeg(p[0], p[1], dblHolder)
      print dblHolder.value
    waitInputConfirm("finished")

def createComps(hostname=socket.gethostname()):
    global ms, adm_svc, rh, rh_svc, seq, seq_svc, armR, armR_svc,  armL, armL_svc, grsp, grsp_svc, servo, log
    ms = rtm.findRTCmanager(hostname)

    rh = rtm.findRTC("RobotHardware0")
    rh_svc = None
    if rh != None:
        rh_svc = OpenHRP.RobotHardwareServiceHelper.narrow(rh.service("service0"))
        servo = rh
        adm = rtm.findRTC("SystemAdmin0")
        if adm != None:
          adm.start()
          adm_svc = OpenHRP.SystemAdminServiceHelper.narrow(adm.service("service0"))
    else:
        rh = rtm.findRTC(bodyinfo.modelName+"Controller(Robot)0")
        servo = rtm.findRTC("PDservo0")
    seq = initRTC("SequencePlayer", "seq")
    seq_svc = OpenHRP.SequencePlayerServiceHelper.narrow(seq.service("service0"))

    armR = initRTC("ArmControl", "armR")
    armR_svc = OpenHRP.ArmControlServiceHelper.narrow(armR.service("service0"))

    armL = initRTC("ArmControl", "armL")
    armL_svc = OpenHRP.ArmControlServiceHelper.narrow(armL.service("service0"))

    grsp = initRTC("Grasper", "grsp")
    grsp_svc = OpenHRP.GrasperServiceHelper.narrow(grsp.service("service0"))

    log = initRTC("DataLogger", "log")

    return [rh, seq, armR, armL, grsp, log]

def connectComps():
    rtm.connectPorts(servo.port("jointStt"),   seq.port("jointStt"))
    rtm.connectPorts(servo.port("jointStt"),   armR.port("jointStt"))
    rtm.connectPorts(servo.port("jointStt"),   armL.port("jointStt"))
    rtm.connectPorts(servo.port("jointStt"),   grsp.port("jointStt"))
    if servo != None:
        rtm.connectPorts(seq.port("jointCmd"),  servo.port("jointCmd"))
        rtm.connectPorts(armR.port("jointCmd"), servo.port("jointCmd"))
        rtm.connectPorts(armL.port("jointCmd"), servo.port("jointCmd"))
        rtm.connectPorts(grsp.port("jointCmd"), servo.port("jointCmd"))
    else: # no forward dynamics mode
        print "no forward dynamics mode is not supported."

def setupLogger():
    global log_svc
    log_svc = OpenHRP.DataLoggerServiceHelper.narrow(log.service("service0"))
    log_svc.add("TimedJointData", "jointStt")
    log_svc.add("TimedJointData", "jointCmd")
    rtm.connectPorts(servo.port("jointStt"), log.port("jointStt"))
    rtm.connectPorts(seq.port("jointCmd"), log.port("jointCmd"))

def saveLog():
    if log_svc == None:
      waitInputConfirm("Setup DataLogger RTC first.")
      return
    log_svc.save('/tmp/sample')
    print 'saved'

def calibrateJoint():
    print('calibrateing joints ...'),
    if rh_svc.calibrateJoint('all') == 0:
      print('finised.')
    else:
      print('failed. execute servoOff() and try again.')

def servoOnHands():
    servoOn('rhand')
    servoOn('lhand')

def servoOffHands():
    servoOff('rhand')
    servoOff('lhand')
    
def EngageProtectiveStop():
    rh_svc.engageProtectiveStop()
  
def DisengageProtectiveStop():
    rh_svc.disengageProtectiveStop()

def reboot():
    waitInputConfirm("Reboot the robot host ?")
    adm_svc.reboot("")

def shutdown():
    waitInputConfirm("Shutdown the robot host ?")
    adm_svc.shutdown("")

#
# move arms using Inverse Kinematics
#
def moveRelativeR(dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
  return moveRelative(armR_svc, dx, dy, dz, dr, dp, dw, rate, wait)

def moveRelativeL(dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
  return moveRelative(armL_svc, dx, dy, dz, dr, dp, dw, rate, wait)

def moveRelative(armsvc, dx, dy, dz, dr, dp, dw, rate, wait):
  x,y,z,r,p,w = getCurerntConfiguration(armsvc)
  return move(x+dx, y+dy, z+dz, r+dr, p+dp, w+dw, rate) 

def moveR(x, y, z, r, p, w, rate=30, wait=True):
  return move(armR_svc, x, y, z, r, p, w, rate, wait)

def moveL(x, y, z, r, p, w, rate=30, wait=True):
  return move(armL_svc, x, y, z, r, p, w, rate, wait)

def move(armsvc, x, y, z, r, p, w, rate, wait):
  ret = armsvc.setTargetAngular(x, y, z, r, p, w, rate)
  while wait and armsvc.checkStatus() == OpenHRP.ArmState.ArmBusyState:
    time.sleep(0.2)
  return ret

def getCurrentConfiguration(armsvc):
  x = org.omg.CORBA.DoubleHolder()
  y = org.omg.CORBA.DoubleHolder()
  z = org.omg.CORBA.DoubleHolder()
  r = org.omg.CORBA.DoubleHolder()
  p = org.omg.CORBA.DoubleHolder()
  w = org.omg.CORBA.DoubleHolder()
  armsvc.getCurrentConfiguration(x, y, z, r, p, w)
  return [x.value, y.value, z.value, r.value, p.value, w.value]

def ikTest():
  import random
  for i in range(10):
    x1 =  0.32
    y1 = -0.20 + 0.1*random.random()
    z1 =  0.10 + 0.1*random.random()
    r1 =  0.00
    p1 = -1.57
    w1 =  0.00
    ret = moveR(x1, y1, z1, r1, p1, w1, 10)
    x2, y2, z2, r2, p2, w2 = getCurrentConfiguration(armR_svc)
    print '\ntarget pos  (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x1, y1, z1, r1, p1, w1)
    print 'current pos (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x2, y2, z2, r2, p2, w2)
    if ret < 0:
      print 'ik failed.'
    else:
      print 'differece   (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x2-x1, y2-y1, z2-z1, r2-r1, p2-p1, w2-w1)

#
# grasper
#
def rhandOpen():
  ttm = org.omg.CORBA.DoubleHolder()
  for i in range(13):
    grsp_svc.setJointAngles('RHAND', anglesFromDistance(i*10)[:4], 1.0, ttm)
    time.sleep(0.1)

def rhandClose():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('RHAND', anglesFromDistance(0)[:4], 1.0, ttm)

def lhandOpen():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', anglesFromDistance(100)[:4], 1.0, ttm)

def lhandClose():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', anglesFromDistance(0)[:4], 1.0, ttm)

def setRHandAnglesDeg(angles):
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('RHAND', [v*math.pi/180.0 for v in angles], 1.0, ttm)

def setLHandAnglesDeg(angles):
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', [v*math.pi/180.0 for v in angles], 1.0, ttm)


def anglesFromDistance(gripDist):
    
    safetyMargin = 3

    l1 = 33
    l2 = 41.9
    l3 = 30
    l4 = l2 - safetyMargin*2
    l5 = 19

    if gripDist < 0.0 or gripDist > (l1+l4)*2:
        return None
    
    xPos   = gripDist/2.0 + safetyMargin
    #print 'xPos =', xPos
    a2Pos  = xPos - l5
    #print 'a2Pos =', a2Pos
    a1radH = math.acos(a2Pos/l2)
    #print 'a1radH = ', a1radH
    a1rad  = math.pi/2.0 - a1radH
    a2rad  = -a1rad
    dEnd   = l2 * math.cos(a1rad) + l3

    return a1rad, a2rad, -a1rad, -a2rad, dEnd


#
# test execution of this script
# e.g: ./sample.py hiro015
#
# if __name__ == '__main__' or __name__ == 'main':
#   if len(sys.argv) > 1:
#     robotHost = sys.argv[1]
#   else:
#     robotHost = None
#   init(robotHost)
#   #goInitial(5)
#   ikTest()

robotHost = socket.gethostname()
init(robotHost)
start_service()
