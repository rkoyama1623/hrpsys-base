#ifndef INTERNALFORCESEPARATOR_H
#define INTERNALFORCESEPARATOR_H

#include <hrpUtil/Eigen3d.h>
#include <map>

struct EndEffectorInfo {
  /**
   * all vector is described in world coordinates
   */
  // inputs
  hrp::Vector3 ref_force, ref_moment;
  hrp::Vector3 abs_force, abs_moment;
  hrp::Vector3 pos;
  hrp::Matrix33 R;
  hrp::Vector3 contact_force_dir;
  // outputs
  hrp::Vector3 ref_force_ex, ref_moment_ex;
  hrp::Vector3 ref_force_in, ref_moment_in;
  hrp::Vector3 ref_force_in_goal, ref_moment_in_goal;
  hrp::Vector3 abs_force_ex, abs_moment_ex;
  hrp::Vector3 abs_force_in, abs_moment_in;
  //
  EndEffectorInfo()
    : ref_force(0,0,0), ref_moment(0,0,0),
      abs_force(0,0,0), abs_moment(0,0,0),
      pos(0,0,0), R(hrp::Matrix33::Identity()),
      contact_force_dir(0,0,0),
      ref_force_ex(0,0,0), ref_moment_ex(0,0,0),
      ref_force_in(0,0,0), ref_moment_in(0,0,0),
      ref_force_in_goal(0,0,0), ref_moment_in_goal(0,0,0),
      abs_force_ex(0,0,0), abs_moment_ex(0,0,0),
      abs_force_in(0,0,0), abs_moment_in(0,0,0)
  {};
};

class InternalForceSeparator {
public:
  InternalForceSeparator();
  void calcInternalForce(std::map<std::string, EndEffectorInfo> &ee_info);
  const bool useMoment(bool use);
  const bool useMoment();
  const bool useQP(bool use);
  const bool useQP();
  bool printp;
  unsigned int debug_level;
protected:
  void calcGraspMatrix(hrp::dmatrix &grasp_matrix, const std::map<std::string, EndEffectorInfo> ee_info);
  void getAbsWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
  void getRefWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
  void calcExWrench(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
#ifdef USE_QPOASES
  void calcExWrenchQPOASES(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
#endif
  int wrench_dim;
  bool use_moment;
  bool use_qp;
};

#endif // INTERNALFORCESEPARATOR_H
