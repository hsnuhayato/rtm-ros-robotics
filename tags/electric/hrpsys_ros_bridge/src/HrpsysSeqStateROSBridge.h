// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.h * @brief hrpsys seq state - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSEQSTATEROSBRIDGE_H
#define HRPSYSSEQSTATEROSBRIDGE_H

#include "HrpsysSeqStateROSBridgeImpl.h"

// rtm
#include <rtm/CorbaNaming.h>

// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "tf/transform_broadcaster.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "hrpsys_ros_bridge/MotorStates.h"
#include "diagnostic_msgs/DiagnosticArray.h"

extern const char* hrpsysseqstaterosbridgeimpl_spec[];

class HrpsysSeqStateROSBridge  : public HrpsysSeqStateROSBridgeImpl
{
 public:
  HrpsysSeqStateROSBridge(RTC::Manager* manager) ;
  ~HrpsysSeqStateROSBridge();

  RTC::ReturnCode_t onInitialize();
  RTC::ReturnCode_t onFinalize();
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void onJointTrajectoryActionGoal();
  void onJointTrajectoryActionPreempt();
  bool sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
		dynamic_reconfigure::Reconfigure::Response &res);

 private:
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub, joint_controller_state_pub, mot_states_pub, diagnostics_pub;
  std::vector<ros::Publisher> fsensor_pub;
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> server;
  ros::ServiceServer sendmsg_srv;
  bool interpolationp;

  tf::TransformBroadcaster br;

  coil::Mutex m_mutex;
  coil::TimeMeasure tm;

  std::string nameserver;
  std::string rootlink_name;

  typedef struct  {
    std::string link_name;
    tf::Transform transform;
  } SensorInfo;
  std::map<std::string, SensorInfo> sensor_info;
};


extern "C"
{
  DLL_EXPORT void HrpsysSeqStateROSBridgeInit(RTC::Manager* manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE_H
