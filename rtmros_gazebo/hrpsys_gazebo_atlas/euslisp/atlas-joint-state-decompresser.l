(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::roseus "atlas-joint-state-decompresser")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-joint-state-compresser-settings.l")

(setq js-decompressed-topic-name "/atlas/joint_states_decompressed")
(setq js-compressed-topic-name "/atlas/joint_states_compressed_buffered")

(setq display-info nil)

(defun get-joint-name (joint-index)
  (if (> (length joint-name-list) joint-index)
      (elt joint-name-list joint-index)
      nil)
)

(defun get-joint-name-float (joint-index)
  (setq joint-index (round joint-index))
  (if (> (length joint-name-list) joint-index)
      (elt joint-name-list joint-index)
      nil)
)

(defun decompress-callback (msg)
  (let ((js-decompressed (instance sensor_msgs::JointState :init))
	joint-names joint-name joint-angles
	)
    (setq hoge msg)
    (send js-decompressed :header (send msg :header))
;;    (dotimes (i (length (send msg :position)))
;;      (setf (elt js-compressed-list joint-index) (elt (send msg :position) i))
      ;;(setq joint-name (get-joint-name (elt (send msg :index) i)))
;;      (setq joint-name (get-joint-name-float (elt (send msg :velocity) i)))
;;      (when joint-name
;;	(push joint-name joint-names)
;;	(push (elt (send msg :position) i) joint-angles)
;;	)
;;      )
    (send js-decompressed :name atlas-joint-name-list)
    (send js-decompressed :position (send msg :position))
    ;;(send js-decompressed :velocity (make-sequence 'float-vector (length joint-names) :initial-element 0))
    ;;(send js-decompressed :effort (make-sequence 'float-vector (length joint-names) :initial-element 0))
    (ros::publish js-decompressed-topic-name js-decompressed)
    (if display-info
	(format t "published ~A~%" js-decompressed-topic-name)
      )
    )
  )

(defun main
  ()
  (joint-index-init)
  (setq js-compressed-list (make-list (length atlas-joint-name-list) :initial-element 0))
  (ros::advertise js-decompressed-topic-name
		  sensor_msgs::JointState 1)
;;  (ros::subscribe js-compressed-topic-name 
;;		  hrpsys_gazebo_atlas::JointStateCompressed #'decompress-callback)
  (ros::subscribe js-compressed-topic-name 
		  sensor_msgs::JointState #'decompress-callback)

  (ros::rate 10)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (unless (ros::ok) (return)))
  )

(main)


