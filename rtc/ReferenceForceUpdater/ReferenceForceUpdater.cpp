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

typedef coil::Guard<coil::Mutex> Guard;

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

    m_qRefIn("qRef", m_qRef),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_rpyIn("rpy", m_rpy),
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
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn",m_baseRpyIn);
  addInPort("rpy",m_rpyIn);

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
  //resie
  qrefv.resize(dof);
  loop = 0;

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
  loop ++;

  // check dataport input
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      if ( m_forceIn[i]->isNew() ) {
          m_forceIn[i]->read();
      }
      if ( m_ref_forceIn[i]->isNew() ) {
          m_ref_forceIn[i]->read();
      }
  }
  if (m_basePosIn.isNew()) {
      m_basePosIn.read();
  }
  if (m_baseRpyIn.isNew()) {
      m_baseRpyIn.read();
  }
  if (m_rpyIn.isNew()) {
      m_rpyIn.read();
  }
  if (m_qRefIn.isNew()) {
      m_qRefIn.read();
  }


  if ( m_qRef.data.length() ==  m_robot->numJoints() ) {
      //check qRef
      if ( DEBUGP ) {
          std::cerr << "[" << m_profile.instance_name << "] qRef = ";
          for ( int i = 0; i <  m_qRef.data.length(); i++ ){
              std::cerr << " " << m_qRef.data[i];
          }
          std::cerr << std::endl;
      }

      Guard guard(m_mutex);

      bool is_active = true;//mode selecter (copy from ic)
      // for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
      //     is_active = is_active || it->second.is_active;
      // }
      if ( !is_active ) {
          // for ( int i = 0; i < m_qRef.data.length(); i++ ){
          //     m_q.data[i] = m_qRef.data[i];
          //     m_robot->joint(i)->q = m_qRef.data[i];
          // }
          //determin ref_force_out from ref_force_in
          for (unsigned int i=0; i<m_ref_force_in.size(); i++){
              for (unsigned int j=0; j<m_ref_force_in.size(); j++)
                  m_ref_force_out[i].data[j] =m_ref_force_in[i].data[j];
              m_ref_forceOut[i]->write();
          }

          return RTC::RTC_OK;
      }

      {
          hrp::dvector qorg(m_robot->numJoints());

          // reference model
          for ( int i = 0; i < m_robot->numJoints(); i++ ){
              qorg[i] = m_robot->joint(i)->q;
              m_robot->joint(i)->q = m_qRef.data[i];
              qrefv[i] = m_qRef.data[i];
          }
          m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
          m_robot->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
          m_robot->calcForwardKinematics();
          if ( (ee_map.find("rleg") != ee_map.end() && ee_map.find("lleg") != ee_map.end()) // if legged robot
               && !use_sh_base_pos_rpy ) {
              // TODO
              //  Tempolarily modify root coords to fix foot pos rot
              //  This will be removed after seq outputs adequate waistRPY discussed in https://github.com/fkanehiro/hrpsys-base/issues/272

              // get current foot mid pos + rot
              std::vector<hrp::Vector3> foot_pos;
              std::vector<hrp::Matrix33> foot_rot;
              std::vector<std::string> leg_names;
              leg_names.push_back("rleg");
              leg_names.push_back("lleg");
              for (size_t i = 0; i < leg_names.size(); i++) {
                  hrp::Link* target_link = m_robot->link(ee_map[leg_names[i]].target_name);
                  foot_pos.push_back(target_link->p + target_link->R * ee_map[leg_names[i]].localPos);
                  foot_rot.push_back(target_link->R * ee_map[leg_names[i]].localR);
              }
              hrp::Vector3 current_foot_mid_pos ((foot_pos[0]+foot_pos[1])/2.0);
              hrp::Matrix33 current_foot_mid_rot;
              rats::mid_rot(current_foot_mid_rot, 0.5, foot_rot[0], foot_rot[1]);
              // calculate fix pos + rot
              hrp::Vector3 new_foot_mid_pos(current_foot_mid_pos);
              hrp::Matrix33 new_foot_mid_rot;
              {
                  hrp::Vector3 ex = hrp::Vector3::UnitX();
                  hrp::Vector3 ez = hrp::Vector3::UnitZ();
                  hrp::Vector3 xv1 (current_foot_mid_rot * ex);
                  xv1(2) = 0.0;
                  xv1.normalize();
                  hrp::Vector3 yv1(ez.cross(xv1));
                  new_foot_mid_rot(0,0) = xv1(0); new_foot_mid_rot(1,0) = xv1(1); new_foot_mid_rot(2,0) = xv1(2);
                  new_foot_mid_rot(0,1) = yv1(0); new_foot_mid_rot(1,1) = yv1(1); new_foot_mid_rot(2,1) = yv1(2);
                  new_foot_mid_rot(0,2) = ez(0); new_foot_mid_rot(1,2) = ez(1); new_foot_mid_rot(2,2) = ez(2);
              }
              // fix root pos + rot to fix "coords" = "current_foot_mid_xx"
              hrp::Matrix33 tmpR (new_foot_mid_rot * current_foot_mid_rot.transpose());
              m_robot->rootLink()->p = new_foot_mid_pos + tmpR * (m_robot->rootLink()->p - current_foot_mid_pos);
              rats::rotm3times(m_robot->rootLink()->R, tmpR, m_robot->rootLink()->R);
              m_robot->calcForwardKinematics();
          }

      }
      //codingstart




      //codingend










  }

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

  //determin ref_force_out from ref_force_in
  for (unsigned int i=0; i<m_ref_force_in.size(); i++){
      for (unsigned int j=0; j<m_ref_force_in.size(); j++)
          m_ref_force_out[i].data[j] =m_ref_force_in[i].data[j];
      m_ref_forceOut[i]->write();
  }

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


