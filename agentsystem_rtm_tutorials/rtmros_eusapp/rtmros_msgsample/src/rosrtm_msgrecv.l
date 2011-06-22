#!/usr/bin/env roseus
(require :rtmeus (format nil "~A/eus/common/rtmeus.l" (unix:getenv 'RTMEUSDIR)))
;;
(defun init-rtc
  nil
  ;;create instance of RTMeus class
  (setq *rtm* (instance RTMeus :init "localhost"))
  ;;load module
  (send *rtm* :load-module (format nil "~A/RTMEus/~A/lib/TimerTest.so" (unix:getenv 'RTMEUSDIR) (unix:getenv "ARCHDIR")) "TimerTestInit")
  ;;create component
  (setq *rtc0* (send *rtm* :create-component "TimerTest" "TimerTest0" :execrate 10 :with-outstate nil))
  ;;
  (unix:sleep 1)
  (send *rtm* :all-activate)
  )
;;
(init-rtc)
;;
(ros::load-ros-manifest "rtmros_msgsample")
(defun recv-cb
  (msg)
  (send *rtc0* :command :servicetest (send msg :data))
  )
;;
(ros::roseus "listener")
(ros::subscribe "/chatter" std_msgs::string #'recv-cb)

;; main
(do-until-key
  (ros::spin-once)
  (ros::sleep)
  )


