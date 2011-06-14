// -*- C++ -*-
/*!
 * @file  TestSend.cpp * @brief TestSend * $Date$ 
 *
 * $Id$ 
 */
#include "TestSend.h"

// Module specification
// <rtc-template block="module_spec">
static const char* helloworldsend_spec[] =
  {
    "implementation_id", "TestSend",
    "type_name",         "TestSend",
    "description",       "TestSend",
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

TestSend::TestSend(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_SendMsgOut("SendMsg", m_SendMsg)

    // </rtc-template>
{
}

TestSend::~TestSend()
{
}


RTC::ReturnCode_t TestSend::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort("SendMsg", m_SendMsgOut);

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
RTC::ReturnCode_t TestSend::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestSend::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "Activated." << std::endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestSend::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestSend::onExecute(RTC::UniqueId ec_id)
{

  m_SendMsg.data.id = 10;
  m_SendMsg.data.msg = "Hello World!";
  m_SendMsgOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestSend::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestSend::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void TestSendInit(RTC::Manager* manager)
  {
    coil::Properties profile(helloworldsend_spec);
    manager->registerFactory(profile,
                             RTC::Create<TestSend>,
                             RTC::Delete<TestSend>);
  }
  
};



