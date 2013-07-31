// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridgeImpl.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridgeImpl.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysseqstaterosbridgeimpl_spec[] =
  {
    "implementation_id", "HrpsysSeqStateROSBridgeImpl",
    "type_name",         "HrpsysSeqStateROSBridgeImpl",
    "description",       "hrpsys seq state - ros bridge",
    "version",           "1.0",
    "vendor",            "Kei Okada",
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

HrpsysSeqStateROSBridgeImpl::HrpsysSeqStateROSBridgeImpl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rsangleIn("rsangle", m_rsangle),
    m_mcangleIn("mcangle", m_mcangle),
    m_gsensorIn("gsensor", m_gsensor),
    m_gyrometerIn("gyrometer", m_gyrometer),
    m_baseTformIn("baseTform", m_baseTform),
    m_rstorqueIn("rstorque", m_rstorque),
    m_servoStateIn("servoState", m_servoState),
    m_mctorqueOut("mctorque", m_mctorque),
    m_SequencePlayerServicePort("SequencePlayerService")

    // </rtc-template>
{
}

HrpsysSeqStateROSBridgeImpl::~HrpsysSeqStateROSBridgeImpl()
{
}


RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rsangle", m_rsangleIn);
  addInPort("mcangle", m_mcangleIn);
  addInPort("gsensor", m_gsensorIn);
  addInPort("gyrometer", m_gyrometerIn);
  addInPort("baseTform", m_baseTformIn);
  addInPort("rstorque", m_rstorqueIn);
  addInPort("servoState", m_servoStateIn);

  // Set OutPort buffer
  addOutPort("mctorque", m_mctorqueOut);

  // Set service provider to Ports

  // Set service consumers to Ports
  m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayerService", m_service0);

  // Set CORBA Service Ports
  addPort(m_SequencePlayerServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  RTC::Properties& prop = getProperties();

  body = new hrp::Body();

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  std::string modelfile =  m_pManager->getConfig ()["model"];

  if (!loadBodyFromModelLoader(body, modelfile.c_str(), 
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
			       )){
    std::cerr << "[HrpsysSeqStateROSBridge] failed to load model[" << prop["model"] << "]" 
	      << std::endl;
  }
  bool ret = false;
  while ( ! ret ) {
    try  {
      bodyinfo = hrp::loadBodyInfo(modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));
      ret = loadBodyFromBodyInfo(body, bodyinfo);
    } catch ( CORBA::SystemException& ex ) {
      std::cerr << "[HrpsysSeqStateROSBridge] CORBA::SystemException " << ex._name() << std::endl;
      sleep(1);
    } catch ( ... ) {
      std::cerr << "[HrpsysSeqStateROSBridge] failed to load model[" << modelfile << "]" << std::endl;;
      sleep(1);
    }
  }


  coil::vstring virtual_force_sensor = coil::split(m_pManager->getConfig()["virtual_force_sensor"], ",");
  int npforce = body->numSensors(hrp::Sensor::FORCE);
  int nvforce = virtual_force_sensor.size()/10;
  int nforce  = npforce + nvforce;
  m_rsforce.resize(nforce);
  m_rsforceIn.resize(nforce);
  for (unsigned int i=0; i<npforce; i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
    m_rsforceIn[i] = new InPort<TimedDoubleSeq>(s->name.c_str(), m_rsforce[i]);
    m_rsforce[i].data.length(6);
    registerInPort(s->name.c_str(), *m_rsforceIn[i]);
    std::cerr << i << " physical force sensor : " << s->name << std::endl;
    m_rsforceName.push_back(s->name);
  }

  for (int j = 0 ; j < body->numSensorTypes(); j++) {
    for (int i = 0 ; i < body->numSensors(j); i++) {
      hrp::Sensor* sensor = body->sensor(j, i);
      SensorInfo si;
      si.transform.setOrigin( tf::Vector3(sensor->localPos(0), sensor->localPos(1), sensor->localPos(2)) );
      hrp::Vector3 rpy = hrp::rpyFromRot(sensor->localR);
      si.transform.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)) );
      OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
      for ( int k = 0; k < links->length(); k++ ) {
        OpenHRP::SensorInfoSequence& sensors = links[k].sensors;
        for ( int l = 0; l < sensors.length(); l++ ) {
          if ( std::string(sensors[l].name) == std::string(sensor->name) ) {
            si.link_name = links[k].segments[0].name;
            sensor_info[sensor->name] = si;
          }
        }
      }
    }
  }


  for(unsigned int j = 0, i = npforce; j < nvforce; j++, i++ ){
    std::string name = virtual_force_sensor[j*10+0];
    std::string base = virtual_force_sensor[j*10+1];
    std::string target = virtual_force_sensor[j*10+2];
    hrp::dvector tr(7);
    for (int k = 0; k < 7; k++ ) {
        coil::stringTo(tr[k], virtual_force_sensor[j*10+3+k].c_str());
    }
    m_rsforceIn[i] = new InPort<TimedDoubleSeq>(name.c_str(), m_rsforce[i]);
    m_rsforce[i].data.length(6);
    registerInPort(name.c_str(), *m_rsforceIn[i]);
    m_rsforceName.push_back(name);

	if ( ! body->link(base) ) {
	  std::cerr << "ERROR : unknown link : " << base << std::endl;
	}
	if ( ! body->link(target) ) {
	  std::cerr << "ERROR : unknown link : " << target << std::endl;
	}

    SensorInfo si;
    si.transform.setOrigin( tf::Vector3(tr[0], tr[1], tr[2]) );
    si.transform.setRotation( tf::Quaternion(tr[3], tr[4], tr[5], tr[6]) );
    OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
    for ( int k = 0; k < links->length(); k++ ) {
      if ( std::string(links[k].name) == target ) {
        si.link_name = links[k].segments[0].name;
      }
    }
    sensor_info[name] = si;

    std::cerr << i << " virtual force sensor : " << name << ": "  << base << "," << target << std::endl;
  }


  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HrpsysSeqStateROSBridgeImplInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysseqstaterosbridgeimpl_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysSeqStateROSBridgeImpl>,
                             RTC::Delete<HrpsysSeqStateROSBridgeImpl>);
  }
  
};


