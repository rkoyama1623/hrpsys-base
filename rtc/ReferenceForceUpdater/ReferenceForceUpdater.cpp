// -*- C++ -*-
/*!
 * @file  ReferenceForceUpdater.cpp
 * @brief Update ReferenceForce
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "util/Hrpsys.h"
#include <boost/assign.hpp>
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
    "conf.default.debugLevel", "0",
    "conf.default.intvec", "1,2,3",
    "conf.default.double", "1.234",
    ""
  };
// </rtc-template>

ReferenceForceUpdater::ReferenceForceUpdater(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_ReferenceForceUpdaterServicePort("ReferenceForceUpdaterService"),

    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qRefIn("qRef", m_qRef),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_rpyIn("rpy", m_rpy),
    m_qOut("q", m_q),
    // </rtc-template>
    m_robot(hrp::BodyPtr()),
    m_debugLevel(0),
    dummy(0)
{
  std::cout << "ReferenceForceUpdater::ReferenceForceUpdater()" << std::endl;
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
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set service provider to Ports
  m_ReferenceForceUpdaterServicePort.registerProvider("service0", "ReferenceForceUpdaterService", m_ReferenceForceUpdaterService);

  // Set service consumers to Ports
  // Set CORBA Service Ports
  addPort(m_ReferenceForceUpdaterServicePort);

  // make m_robot instance
  RTC::Properties& prop = getProperties();//get properties information from .wrl file
  coil::stringTo(m_dt, prop["dt"].c_str());
  m_robot = hrp::BodyPtr(new hrp::Body());
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), //load robot model for m_robot
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
  }

  // Setting for wrench data ports (real + virtual)
  std::vector<std::string> fsensor_names;
  //   find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  // load virtual force sensors
  readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
  int nvforce = m_vfs.size();
  for (unsigned int i=0; i<nvforce; i++){
      for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
          if (it->second.id == i) fsensor_names.push_back(it->first);
      }
  }

  //   add ports for all force sensors (real force, input and output of ref_force)
  int nforce  = npforce + nvforce;
  m_force.resize(nforce);
  m_forceIn.resize(nforce);
  m_ref_force_in.resize(nforce);
  m_ref_force_out.resize(nforce);
  m_ref_forceIn.resize(nforce);
  m_ref_forceOut.resize(nforce);
  std::cerr << "[" << m_profile.instance_name << "] create force sensor ports" << std::endl;
  for (unsigned int i=0; i<nforce; i++){
      // actual inport
      m_forceIn[i] = new InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
      m_force[i].data.length(6);
      registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
      // ref inport
      m_ref_force_in[i].data.length(6);
      for (unsigned int j=0; j<6; j++) m_ref_force_in[i].data[j] = 0.0;
      m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force_in[i]);
      registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
      // ref Outport
      m_ref_force_out[i].data.length(6);
      for (unsigned int j=0; j<6; j++) m_ref_force_out[i].data[j] = 0.0;
      m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"Out").c_str(), m_ref_force_out[i]);
      registerOutPort(std::string("ref_"+fsensor_names[i]+"Out").c_str(), *m_ref_forceOut[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
  }

  // alocate memory for force and moment vector represented in absolute coordinates
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
  }

  // check if the dof of m_robot match the number of joint in m_robot
  unsigned int dof = m_robot->numJoints();
  for ( int i = 0 ; i < dof; i++ ){
      if ( i != m_robot->joint(i)->jointId ) {
          std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
          return RTC::RTC_ERROR;
      }
  }

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


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t ReferenceForceUpdater::onExecute(RTC::UniqueId ec_id)
{
  static int loop = 0;
  loop ++;

  /*
    ToDo
    - ポート接続の書き換えをする。
    - icの中で書き換えたバージョン
    https://github.com/rkoyama1623/hrpsys-base/tree/update_ref_force_stable?files=1
    のコピペをして、毎週期更新にする。
    - seqplayerのインスタンスを追加
    - 更新周期を間引くようにする。
    - 更新するときのゲインなどをeusから書き換えられるように、idlを編集
   */

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


