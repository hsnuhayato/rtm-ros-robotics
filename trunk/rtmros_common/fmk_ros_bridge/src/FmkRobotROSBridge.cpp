// -*- C++ -*-
/*!
 * @file  FmkRobotROSBridge.cpp * @brief VehicleService * $Date$ 
 *
 * $Id$ 
 */
#include "FmkRobotROSBridge.h"
// Module specification
// <rtc-template block="module_spec">
static const char* fmkrobotrosbridge_spec[] =
  {
    "implementation_id", "FmkRobotROSBridge",
    "type_name",         "FmkRobotROSBridge",
    "description",       "VehicleService",
    "version",           "1.0.0",
    "vendor",            "",
    "category",          "",
    "activity_type",     "PERIODIC",
    "kind",              "VehicleServiceConsumer",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

FmkRobotROSBridge::FmkRobotROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_VehicleServicePort("VehicleService")
    // </rtc-template>
{

}

FmkRobotROSBridge::~FmkRobotROSBridge()
{
}


RTC::ReturnCode_t FmkRobotROSBridge::onInitialize()
{
  // Parameter velocity/acceleration
  ros::param::param<double>("~max_vx", max_vx, 0.3);
  ros::param::param<double>("~max_vw", max_vw, 0.5);
  ros::param::param<double>("~max_ax", max_ax, 0.3);
  ros::param::param<double>("~max_aw", max_aw, 0.5);
  // ROS msg/srv port
  odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  velocity_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &FmkRobotROSBridge::onVelocityCommand, this);

  halt_srv = nh.advertiseService("halt_motors",
				 &FmkRobotROSBridge::onHaltMotorService, this);
  reset_srv = nh.advertiseService("reset_motors",
				  &FmkRobotROSBridge::onResetMotorService, this);

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports
  m_VehicleServicePort.registerConsumer("vehicleService", "VehicleService", m_service0);

  // Set CORBA Service Ports
  addPort(m_VehicleServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t FmkRobotROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t FmkRobotROSBridge::onExecute(RTC::UniqueId ec_id)
{
  static ros::Time last_verbose = ros::Time(0);
  ros::Duration elapsed = ros::Time::now() - last_verbose;
  bool verbose = (elapsed.toSec() > 1.0);
  if(verbose) last_verbose = ros::Time::now();

  try {

    if(verbose) {
      double voltage;
      if( m_service0->getBatteryVoltage(voltage) )
	std::cout << "voltage = " << voltage << std::endl;

      char* version = CORBA::string_alloc(1024); // this is for CORBA::String_out
      if( m_service0->getIFVersion(version) )
	std::cout << "version = " << version << std::endl;
      CORBA::string_free(version);

      short val;
      char* state = CORBA::string_alloc(1024); // this is for CORBA::String_out
      if( m_service0->getState(val,state) )
	std::cout << "state = " << state << std::endl;
      CORBA::string_free(state);

      short seqsize=16, alarmnum;
      SeqAlarm* palarms;
      if( m_service0->getActiveAlarm(seqsize, alarmnum, palarms) ) {
	for(int cnt=0; cnt < (*palarms).length(); cnt++) {
	  std::cout << "  alarm[" << cnt+1 << "]:"
		    << "code=" << (*palarms)[cnt].code << ", "
		    << "type=" << (*palarms)[cnt].type << ", "
		    << "src=" << (*palarms)[cnt].source << std::endl;
	}
      }
    }

    Position pos;
    if( m_service0->getPosition(pos) ) {
      if(verbose) {
	std::cout << "pos = (" << pos.x  << "[mm],"
		  << pos.y << "[mm]," << pos.theta << "[deg])"
		  << std::endl;
      }
      //
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "base_footprint";
      odom.pose.pose.position.x = pos.x / 1000.0;
      odom.pose.pose.position.y = pos.y / 1000.0;
      odom.pose.pose.orientation.w = cos((pos.theta*(M_PI/180.0))/2.0);
      odom.pose.pose.orientation.z = sin((pos.theta*(M_PI/180.0))/2.0);
      odometry_pub.publish(odom);
      //
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(odom.pose.pose.position.x,
				       odom.pose.pose.position.y,
				       0.0) );
      transform.setRotation( tf::Quaternion(0, 0,
					    odom.pose.pose.orientation.z,
					    odom.pose.pose.orientation.w) );
      tf_pub.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "/odom", "/base_footprint"));
    }
  }
  catch (...) {
    std::cout << "No service connected." << std::endl;
  }

  //calling all callbacks
  while(!ros::getGlobalCallbackQueue()->isEmpty())
    ros::spinOnce();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t FmkRobotROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t FmkRobotROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void FmkRobotROSBridge::onVelocityCommand(const geometry_msgs::TwistConstPtr& msg) {
  //latest_v = ros::Time::now();
  velocity = geometry_msgs::Twist(*msg);
  //
  try {
    m_service0->setJogTimeout(100); // command interval < 100[msec].
    if( !m_service0->moveJog(velocity.linear.x * 1000.0, // m -> mm
			     velocity.linear.y * 1000.0, // m -> mm
			     velocity.angular.z * 180.0 / M_PI) ) {
      ROS_WARN("commanded velocity is invalid");
    }
  }
  catch (...) {
    ROS_ERROR("Error occur in onCommandVelocity");
  }
}
bool FmkRobotROSBridge::onHaltMotorService(std_srvs::Empty::Request  &req,
					   std_srvs::Empty::Response &res) {
  m_service0->stop();
  m_service0->setServo(false);
  m_service0->setPower(false);

  return true;
}

bool FmkRobotROSBridge::onResetMotorService(std_srvs::Empty::Request  &req,
					    std_srvs::Empty::Response &res) {

  // check velocity/acceleration
  Velocity vel;
  m_service0->getVelocityLimit(vel.translation, vel.rotation);
  vel.translation = std::min(vel.translation, max_vx * 1000);
  vel.rotation    = std::min(vel.rotation,    max_vw * 180.0 / M_PI);
  m_service0->setVelocity(vel);
  Acc acc;
  m_service0->getAccelerationLimit(acc.translation, acc.rotation);
  acc.translation = std::min(acc.translation, max_ax * 1000);
  acc.rotation    = std::min(acc.rotation,    max_aw * 180.0 / M_PI);
  m_service0->setAcceleration(acc);

  // start
  m_service0->clearAlarm();
  m_service0->unlock();
  m_service0->setPower(true);
  m_service0->setServo(true);

  return true;
}



extern "C"
{

  void FmkRobotROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(fmkrobotrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<FmkRobotROSBridge>,
                             RTC::Delete<FmkRobotROSBridge>);
  }

};
