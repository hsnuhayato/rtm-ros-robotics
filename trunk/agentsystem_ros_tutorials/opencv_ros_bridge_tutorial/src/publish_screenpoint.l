#!/usr/bin/env roseus

(ros::load-ros-manifest "opencv_ros_bridge_tutorial")
(ros::roseus "publish_screenpoint")
(ros::advertise "/image_painted/screenpoint" geometry_msgs::PointStamped 1)
(setq i 0)
(ros::rate 10)
(while (ros::ok)
 (setq msg (instance geometry_msgs::PointStamped :init))
 (send msg :header :stamp (ros::time-now))
 (send msg :point :x (+ 320 (* 180 (sin i))))
 (send msg :point :y (+ 240 (* 180 (cos i))))
 (incf i 0.1)
 (ros::ros-info "msg ~A ~A" (send msg :point :x) (send msg :point :y))
 (ros::publish "/image_painted/screenpoint" msg)
 (ros::sleep))


