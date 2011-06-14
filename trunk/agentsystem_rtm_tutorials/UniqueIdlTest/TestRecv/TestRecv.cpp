// -*- C++ -*-
/*!
 * @file  TestRecv.cpp * @brief TestRecv * $Date$ 
 *
 * $Id$ 
 */
#include "TestRecv.h"

// Module specification
// <rtc-template block="module_spec">
static const char* helloworldsend_spec[] =
  {
    "implementation_id", "TestRecv",
    "type_name",         "TestRecv",
    "description",       "TestRecv",
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

TestRecv::TestRecv(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RecvMsgIn("RecvMsg", m_RecvMsg)

    // </rtc-template>
{
}

TestRecv::~TestRecv()
{
}


RTC::ReturnCode_t TestRecv::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("RecvMsg", m_RecvMsgIn);

  // Set OutPort buffer

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
RTC::ReturnCode_t TestRecv::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestRecv::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "Activated." << std::endl;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestRecv::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TestRecv::onExecute(RTC::UniqueId ec_id)
{
  if(m_RecvMsgIn.isNew()){
    m_RecvMsgIn.read();
    
    fprintf(stderr, "id=%d msg=%s\n", m_RecvMsg.data.id, (char*)m_RecvMsg.data.msg);
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TestRecv::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t TestRecv::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void TestRecvInit(RTC::Manager* manager)
  {
    coil::Properties profile(helloworldsend_spec);
    manager->registerFactory(profile,
                             RTC::Create<TestRecv>,
                             RTC::Delete<TestRecv>);
  }
  
};



