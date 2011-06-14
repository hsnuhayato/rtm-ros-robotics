// -*- C++ -*-
/*!
 * @file  TestVelocity2DSend.cpp * @brief TestVelocity2DSend * $Date$ 
 *
 * $Id$ 
 */
#include "TestVelocity2DSend.h"

// Module specification
// <rtc-template block="module_spec">
static const char* helloworldsend_spec[] =
  {
    "implementation_id", "TestVelocity2DSend",
    "type_name",         "TestVelocity2DSend",
    "description",       "TestVelocity2DSend",
    "version",           "1.0",
    "vendor",            "JSK",
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

TestVelocity2DSend::TestVelocity2DSend(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_SendVelOut("SendVel", m_SendVel)

    // </rtc-template>
{
}

TestVelocity2DSend::~TestVelocity2DSend()
{
}


RTC::ReturnCode_t TestVelocity2DSend::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("SendVel", m_SendVelOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  std::cerr << "Initializing.." << std::endl;
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t TestVelocity2DSend::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestVelocity2DSend::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "Activated." << std::endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestVelocity2DSend::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestVelocity2DSend::onExecute(RTC::UniqueId ec_id)
{

  m_SendVel.data.vx = 1.5;
  m_SendVel.data.vy = -0.5;
  m_SendVel.data.va = 0.1;
  m_SendVelOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestVelocity2DSend::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestVelocity2DSend::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void TestVelocity2DSendInit(RTC::Manager* manager)
  {
    coil::Properties profile(helloworldsend_spec);
    manager->registerFactory(profile,
                             RTC::Create<TestVelocity2DSend>,
                             RTC::Delete<TestVelocity2DSend>);
  }
  
};



