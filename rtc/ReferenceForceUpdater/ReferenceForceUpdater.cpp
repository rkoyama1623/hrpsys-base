// -*- C++ -*-
/*!
 * @file  ReferenceForceUpdater.cpp
 * @brief Update ReferenceForce
 * $Date$
 *
 * $Id$
 */

#include "ReferenceForceUpdater.h"
#include "util/VectorConvert.h"

// Module specification
// <rtc-template block="module_spec">
static const char* ReferenceForceUpdater_spec[] =
  {
    "implementation_id", "ReferenceForceUpdater",
    "type_name",         "ReferenceForceUpdater",
    "description",       "update reference force",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.string", "test",
    "conf.default.intvec", "1,2,3",
    "conf.default.double", "1.234",

    ""
  };
// </rtc-template>

ReferenceForceUpdater::ReferenceForceUpdater(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_dataIn("dataIn", m_data),
    m_dataOut("dataOut", m_data),
    m_NullServicePort("NullService"),
    // </rtc-template>
	dummy(0)
{
  std::cout << "ReferenceForceUpdater::ReferenceForceUpdater()" << std::endl;
  m_data.data = 0;
}

ReferenceForceUpdater::~ReferenceForceUpdater()
{
  std::cout << "ReferenceForceUpdater::~ReferenceForceUpdater()" << std::endl;
}



RTC::ReturnCode_t ReferenceForceUpdater::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("string", confstring, "testtest");
  bindParameter("intvec", confintvec, "4,5,6,7");
  bindParameter("double", confdouble, "4.567");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("dataIn", m_dataIn);

  // Set OutPort buffer
  addOutPort("dataOut", m_dataOut);
  
  // Set service provider to Ports
  m_NullServicePort.registerProvider("service0", "NullService", m_NullService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_NullServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  std::cout << "prop[\"testconf\"] = " << prop["testconf"] << std::endl;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ReferenceForceUpdater::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ReferenceForceUpdater::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ReferenceForceUpdater::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ReferenceForceUpdater::onExecute(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << "), data = " << m_data.data << std::endl;
  std::cout << "confstring = " << confstring << std::endl;
  std::cout << "confintvec = ";
  for (unsigned int i=0; i<confintvec.size(); i++){
      std::cout << confintvec[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "confdouble = " << confdouble << std::endl;

  while (m_dataIn.isNew()){
      m_dataIn.read();
      std::cout << m_profile.instance_name << ": read(), data = " << m_data.data << std::endl;
  }
  m_data.data += 1;

  m_dataOut.write();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ReferenceForceUpdater::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void ReferenceForceUpdaterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(ReferenceForceUpdater_spec);
    manager->registerFactory(profile,
                             RTC::Create<ReferenceForceUpdater>,
                             RTC::Delete<ReferenceForceUpdater>);
  }

};


