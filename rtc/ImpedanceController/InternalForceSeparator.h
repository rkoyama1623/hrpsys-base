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
  // outputs
  hrp::Vector3 ref_force_ex, ref_moment_ex;
  hrp::Vector3 ref_force_in, ref_moment_in;
  hrp::Vector3 abs_force_ex, abs_moment_ex;
  hrp::Vector3 abs_force_in, abs_moment_in;
};

class InternalForceSeparator {
public:
  InternalForceSeparator();
  void calcInternalForce(std::map<std::string, EndEffectorInfo> &ee_info);
  const bool useMoment(bool use);
  const bool useMoment();
  bool printp;
  unsigned int debug_level;
protected:
  void calcGraspMatrix(hrp::dmatrix &grasp_matrix, const std::map<std::string, EndEffectorInfo> ee_info);
  void getAbsWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
  void getRefWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info);
  void calcExWrench(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench);
  int wrench_dim;
  bool use_moment;
};

#endif // INTERNALFORCESEPARATOR_H
