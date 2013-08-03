// -*- C++ -*-
/*!
 * @file  HrpsysJointTrajectoryBridge.cpp * @brief hrpsys setJointAngle - ros joint trajectory bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysJointTrajectoryBridge.h"

#include "rtm/idl/RTC.hh"
#include "hrpsys_ros_bridge/idl/ExecutionProfileService.hh"
#include "hrpsys_ros_bridge/idl/RobotHardwareService.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysjointtrajectorybridge_spec[] =
  {
    "implementation_id", "HrpsysJointTrajectoryBridge",
    "type_name",         "HrpsysJointTrajectoryBridge",
    "description",       "hrpsys setJointAngle - ros joint trajectory bridge",
    "version",           "1.0",
    "vendor",            "Yohei Kakiuchi",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

HrpsysJointTrajectoryBridge::HrpsysJointTrajectoryBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_SequencePlayerServicePort("SequencePlayerService")
    // </rtc-template>
{
}

HrpsysJointTrajectoryBridge::~HrpsysJointTrajectoryBridge()
{
}


RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onInitialize()
{
  m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayerService", m_service0);

  addPort(m_SequencePlayerServicePort);

  RTC::Properties& prop = getProperties();

  body = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0) {
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  std::string modelfile =  m_pManager->getConfig ()["model"];

  if (!loadBodyFromModelLoader(body, modelfile.c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()))){
    std::cerr << "[HrpsysJointTrajectoryBridge] failed to load model[" << prop["model"] << "]"
              << std::endl;
  }
  bool ret = false;
  while ( ! ret ) {
    try  {
      bodyinfo = hrp::loadBodyInfo(modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));
      ret = loadBodyFromBodyInfo(body, bodyinfo);
    } catch ( CORBA::SystemException& ex ) {
      std::cerr << "[HrpsysJointTrajectoryBridge] CORBA::SystemException " << ex._name() << std::endl;
      sleep(1);
    } catch ( ... ) {
      std::cerr << "[HrpsysJointTrajectoryBridge] failed to load model[" << modelfile << "]" << std::endl;;
      sleep(1);
    }
  }

  if (body == NULL) {
    return RTC::RTC_ERROR;
  }
  body->calcForwardKinematics();

  ROS_INFO_STREAM("[HrpsysJointTrajectoryBridge] @Initilize name : " << getInstanceName() << " done");

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onActivated(RTC::UniqueId ec_id) {
  // ROS_INFO("ON_ACTIVATED");
  std::string config_name;
  config_name = nh.resolveName("controller_configuration");
  if (nh.hasParam(config_name)) {
    XmlRpc::XmlRpcValue param_val;
    nh.getParam(config_name, param_val);
    if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < param_val.size(); i++) {
        if (param_val[i].getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
          XmlRpc::XmlRpcValue gval = param_val[i]["group_name"];
          XmlRpc::XmlRpcValue cval = param_val[i]["controller_name"];
          XmlRpc::XmlRpcValue lval = param_val[i]["joint_list"];

          std::string gname = gval;
          std::string cname = cval;
          std::vector<std::string> jlst;
          if (lval.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for(int s = 0; s < lval.size(); s++) {
              jlst.push_back(lval[s]);
            }
          }
          if(gname.length() == 0 && cname.length() > 0) {
            gname = cname;
          }
          if(gname.length() > 0 && cname.length() == 0) {
            cname = gname;
          }
          if(gname.length() > 0 && cname.length() > 0 && jlst.size() > 0) {
            ROS_INFO_STREAM("ADD_GROUP: " << gname << " (" << cname << ")");
            ROS_INFO_STREAM("  JOINT:");
            for(size_t s = 0; s < jlst.size(); s++) {
              ROS_INFO_STREAM("    " << jlst[s]);
            }
            jointTrajectoryActionObj::Ptr tmp (new jointTrajectoryActionObj (this, cval, gval, jlst));
            trajectory_actions.push_back(tmp);
          }
        }
      }
    } else {
      ROS_WARN_STREAM("param: " << config_name << ", configuration is not an array.");
    }
  } else {
    ROS_WARN_STREAM("param: " << config_name << ", param does not exist.");
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onFinalize()
{
  // delete objs
  for(size_t i = 0; i < trajectory_actions.size(); i++) {
    trajectory_actions[i].reset();
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onExecute(RTC::UniqueId ec_id)
{
  ros::spinOnce();
  for (size_t i = 0; i < trajectory_actions.size(); i++) {
    trajectory_actions[i]->proc();
  }

  return RTC::RTC_OK;
}

HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
jointTrajectoryActionObj(HrpsysJointTrajectoryBridge *ptr,
                         std::string &cname, std::string &gname, std::vector<std::string> &jlist) {
  parent = ptr;
  controller_name = cname;
  groupname = gname;
  joint_list = jlist;

  joint_trajectory_server.reset
    (new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>
     (parent->nh, controller_name + "/joint_trajectory_action", false));
  follow_joint_trajectory_server.reset
    (new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
     (parent->nh, controller_name + "/follow_joint_trajectory_action", false));

  joint_trajectory_server->
    registerGoalCallback(boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj
                                     ::onJointTrajectoryActionGoal, this));
  joint_trajectory_server->
    registerPreemptCallback(boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj
                                        ::onJointTrajectoryActionPreempt, this));
  follow_joint_trajectory_server->
    registerGoalCallback(boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj
                                     ::onFollowJointTrajectoryActionGoal, this));
  follow_joint_trajectory_server->
    registerPreemptCallback(boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj
                                        ::onFollowJointTrajectoryActionPreempt, this));

  joint_controller_state_pub = parent->nh.
    advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(controller_name + "/state", 1);

  if (groupname.length() > 0) {
    OpenHRP::SequencePlayerService::StrSequence jnames;
    jnames.length(joint_list.size());
    for(size_t i = 0; i < joint_list.size(); i++) {
      jnames[i] = joint_list[i].c_str();
    }
    try {
        parent->m_service0->addJointGroup(groupname.c_str(), jnames);
    } catch ( CORBA::SystemException& ex ) {
      std::cerr << "[HrpsysJointTrajectoryBridge] CORBA::SystemException " << ex._name() << std::endl;
      sleep(1);
    } catch ( ... ) {
      std::cerr << "[HrpsysJointTrajectoryBridge] failed to addJointGroup[" << groupname.c_str() << "]" << std::endl;;
      sleep(1);
    }
  }

  interpolationp = false;

  joint_trajectory_server->start();
  follow_joint_trajectory_server->start();
}

HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::~jointTrajectoryActionObj() {
  if ( joint_trajectory_server->isActive() ) {
    joint_trajectory_server->setPreempted();
  }

  if ( follow_joint_trajectory_server->isActive() ) {
    follow_joint_trajectory_server->setPreempted();
  }

  joint_trajectory_server->shutdown();
  follow_joint_trajectory_server->shutdown();
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
proc() {
  // finish interpolation
  if (joint_trajectory_server->isActive() &&
      interpolationp == true && parent->m_service0->isEmpty() == true) {
    pr2_controllers_msgs::JointTrajectoryResult result;
    joint_trajectory_server->setSucceeded(result);
    interpolationp = false;
  }
  if (follow_joint_trajectory_server->isActive() &&
      interpolationp == true && parent->m_service0->isEmpty() == true) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    follow_joint_trajectory_server->setSucceeded(result);
    interpolationp = false;
  }
#if 0
  ros::Time tm_on_execute = ros::Time::now();

  control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
  follow_joint_trajectory_feedback.header.stamp = tm_on_execute;

  if (!follow_joint_trajectory_feedback->joint_names.empty() &&
      !follow_joint_trajectory_feedback->actual.positions.empty()) {
    follow_joint_trajectory_server->publishFeedback(follow_joint_trajectory_feedback);
  }
#endif
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
onJointTrajectory(trajectory_msgs::JointTrajectory trajectory) {
  parent->m_mutex.lock();

  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ")");
  // TODO: check size and joint names

  OpenHRP::dSequenceSequence angles;
  OpenHRP::dSequence duration;

  angles.length(trajectory.points.size()) ;
  duration.length(trajectory.points.size()) ;

  std::vector<std::string> joint_names = trajectory.joint_names;

  for (unsigned int i=0; i < trajectory.points.size(); i++) {
    angles[i].length(joint_names.size());

    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
    for (unsigned int j=0; j < joint_names.size(); j++) {
      ROS_INFO_STREAM("b_j: " << joint_names[j] << " -> " << point.positions[j]);
      parent->body->link(joint_names[j])->q = point.positions[j];
    }

    parent->body->calcForwardKinematics();

#if 0 // fullbody controller
    int j = 0;
    std::vector<hrp::Link*>::const_iterator it = parent->body->joints().begin();
    while ( it != parent->body->joints().end() ) {
      hrp::Link* l = ((hrp::Link*)*it);
      angles[i][j] = l->q;
      ++it;++j;
    }
#endif
    for (unsigned int j=0; j < joint_names.size(); j++) {
      ROS_INFO_STREAM("a_j: " << joint_names[j] << " -> " << point.positions[j]);
      angles[i][j] = parent->body->link(joint_names[j])->q;
    }

    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ") : time_from_start " << trajectory.points[i].time_from_start.toSec());

    if ( i > 0 ) {
      duration[i] =  trajectory.points[i].time_from_start.toSec() - trajectory.points[i-1].time_from_start.toSec();
    } else { // if i == 0
      if ( trajectory.points.size()== 1 ) {
        duration[i] = trajectory.points[i].time_from_start.toSec();
      } else { // 0.2 is magic number writtein in roseus/euslisp/robot-interface.l
        duration[i] = trajectory.points[i].time_from_start.toSec() - 0.2;
      }
    }
  }

  parent->m_mutex.unlock();
  if ( duration.length() == 1 ) {
    if (groupname.length() > 0) { // group
      ROS_INFO_STREAM("setJointAnglesOfGroup: " << groupname);
      parent->m_service0->setJointAnglesOfGroup (groupname.c_str(), angles[0], duration[0]);
    } else {  // fullbody
      parent->m_service0->setJointAngles(angles[0], duration[0]);
    }
  } else {
    if (groupname.length() > 0) { // group
      ROS_INFO_STREAM("playPatternGroup: " << groupname);
      parent->m_service0->playPatternOfGroup(groupname.c_str(), angles, duration);
    } else {
      OpenHRP::dSequenceSequence rpy, zmp;
      parent->m_service0->playPattern(angles, rpy, zmp, duration);
    }
  }

  interpolationp = true;
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
onJointTrajectoryActionGoal() {
  pr2_controllers_msgs::JointTrajectoryGoalConstPtr
    goal = joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
onFollowJointTrajectoryActionGoal() {
  control_msgs::FollowJointTrajectoryGoalConstPtr
    goal = follow_joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
onJointTrajectoryActionPreempt() {
  joint_trajectory_server->setPreempted();
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::
onFollowJointTrajectoryActionPreempt() {
  follow_joint_trajectory_server->setPreempted();
}

extern "C"
{
 
  void HrpsysJointTrajectoryBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysjointtrajectorybridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysJointTrajectoryBridge>,
                             RTC::Delete<HrpsysJointTrajectoryBridge>);
  }
  
};