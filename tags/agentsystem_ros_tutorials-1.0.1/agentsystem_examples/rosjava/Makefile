ROSJAVA_CORE=`rosstack find rosjava_core`

all:
	cd ${ROSJAVA_CORE}/rosjava_tutorial_pubsub && ROS_PACKAGE_PATH=/opt/ros/fuerte/share:/opt/ros/fuerte/stacks ${ROSJAVA_CORE}/gradlew installApp
	cd ${ROSJAVA_CORE}/rosjava_tutorial_services && ROS_PACKAGE_PATH=/opt/ros/fuerte/share:/opt/ros/fuerte/stacks ${ROSJAVA_CORE}/gradlew installApp

talker:
	${ROSJAVA_CORE}/rosjava_tutorial_pubsub/build/install/rosjava_tutorial_pubsub/bin/rosjava_tutorial_pubsub org.ros.rosjava_tutorial_pubsub.Talker

listener:
	${ROSJAVA_CORE}/rosjava_tutorial_pubsub/build/install/rosjava_tutorial_pubsub/bin/rosjava_tutorial_pubsub org.ros.rosjava_tutorial_pubsub.Listener

add_two_ints_server:
	${ROSJAVA_CORE}/rosjava_tutorial_services/build/install/rosjava_tutorial_services/bin/rosjava_tutorial_services org.ros.rosjava_tutorial_services.Server

add_two_ints_client:
	${ROSJAVA_CORE}/rosjava_tutorial_services/build/install/rosjava_tutorial_services/bin/rosjava_tutorial_services org.ros.rosjava_tutorial_services.Client
