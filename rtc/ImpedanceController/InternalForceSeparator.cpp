#include <hrpUtil/MatrixSolvers.h>
#include <iostream>
#include "InternalForceSeparator.h"

# define PRINT_VECTOR(vec) (vec.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")))

InternalForceSeparator::InternalForceSeparator() : printp(false) {};
void InternalForceSeparator::calcInternalForce(std::map<std::string, EndEffectorInfo> &ee_info) {
  /**
   * get ee_info and set ee_info[limb].xxx_ex, ee_info[limb].xxx_in,
   */
  hrp::dmatrix grasp_matrix = hrp::dmatrix::Zero(6,12);
  hrp::dvector wrench_abs=hrp::dvector::Zero(12);
  hrp::dvector wrench_ref=hrp::dvector::Zero(12);
  hrp::dvector wrench_abs_ex=hrp::dvector::Zero(12);
  hrp::dvector wrench_abs_in=hrp::dvector::Zero(12);
  hrp::dvector wrench_ref_ex=hrp::dvector::Zero(12);
  hrp::dvector wrench_ref_in=hrp::dvector::Zero(12);
  // calc wrenches
  calcGraspMatrix(grasp_matrix, ee_info);
  getAbsWrench(wrench_abs, ee_info);
  getRefWrench(wrench_ref, ee_info);
  calcExWrench(wrench_abs_ex, grasp_matrix, wrench_abs);
  calcExWrench(wrench_ref_ex, grasp_matrix, wrench_ref);
  wrench_abs_in = wrench_abs - wrench_abs_ex;
  wrench_ref_in = wrench_ref - wrench_ref_ex;
  // debug print
  if (printp && debug_level > 1) {
    std::cerr << "grasp matrix:" << std::endl <<grasp_matrix << std::endl;
    std::cerr << "abs wrench:"<< PRINT_VECTOR(wrench_abs) << std::endl;
    std::cerr << "abs external wrench:" << PRINT_VECTOR(wrench_abs_ex) << std::endl;
    std::cerr << "abs internal wrench:" << PRINT_VECTOR(wrench_abs_in) << std::endl;
    std::cerr << "ref wrench:"<< PRINT_VECTOR(wrench_ref) << std::endl;
    std::cerr << "ref external wrench:" << PRINT_VECTOR(wrench_ref_ex) << std::endl;
    std::cerr << "ref internal wrench:" << PRINT_VECTOR(wrench_ref_in) << std::endl;
  }
  // set output
  for ( std::map<std::string, EndEffectorInfo>::iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    iter->second.ref_force_ex = wrench_ref_ex.segment<3>(0 + 6*limb_index);
    iter->second.ref_moment_ex = wrench_ref_ex.segment<3>(3 + 6*limb_index);
    iter->second.ref_force_in = wrench_ref_in.segment<3>(0 + 6*limb_index);
    iter->second.ref_moment_in = wrench_ref_in.segment<3>(3 + 6*limb_index);
    iter->second.abs_force_ex = wrench_abs_ex.segment<3>(0 + 6*limb_index);
    iter->second.abs_moment_ex = wrench_abs_ex.segment<3>(3 + 6*limb_index);
    iter->second.abs_force_in = wrench_abs_in.segment<3>(0 + 6*limb_index);
    iter->second.abs_moment_in = wrench_abs_in.segment<3>(3 + 6*limb_index);
  }
  // debug print
  if (printp) {
    for ( std::map<std::string, EndEffectorInfo>::iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
      std::cerr << iter->first << ":"<< std::endl;
      std::cerr << "ref_force_ex: " << PRINT_VECTOR(iter->second.ref_force_ex) << std::endl;
      std::cerr << "ref_force_in: " << PRINT_VECTOR(iter->second.ref_force_in) << std::endl;
      std::cerr << "abs_force_ex: " << PRINT_VECTOR(iter->second.abs_force_ex) << std::endl;
      std::cerr << "abs_force_in: " << PRINT_VECTOR(iter->second.abs_force_in) << std::endl;
    }
  }
};

void InternalForceSeparator::calcGraspMatrix(hrp::dmatrix &grasp_matrix, const std::map<std::string, EndEffectorInfo> ee_info) {
  /**
   * return G = \matrix[]
   * \f[
   * \boldsymbol{G} = 
   * \begin{bmatrix}
   * I           & O & I            & O \\
   * r_{0}\times & I & r_{1} \times & I
   * \end{bmatrix}
   * \f]
   */
  grasp_matrix = hrp::dmatrix::Zero(6,12);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    grasp_matrix.block<3,3>(0,0+limb_index*6) = hrp::dmatrix::Identity(3,3);
    grasp_matrix.block<3,3>(3,3+limb_index*6) = hrp::dmatrix::Identity(3,3);
    grasp_matrix.block<3,3>(3,0+limb_index*6) = hrp::hat(iter->second.pos);
  }
};

void InternalForceSeparator::getAbsWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  wrench = hrp::dvector::Zero(12);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    wrench.segment<3>(0+limb_index*6) = iter->second.abs_force;
    wrench.segment<3>(3+limb_index*6) = iter->second.abs_moment;
  }
};

void InternalForceSeparator::getRefWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  wrench = hrp::dvector::Zero(12);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    wrench.segment<3>(0+limb_index*6) = iter->second.ref_force;
    wrench.segment<3>(3+limb_index*6) = iter->second.ref_moment;
  }
};

void InternalForceSeparator::calcExWrench(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench) {
  hrp::dmatrix gmat_inv = hrp::dmatrix::Zero(gmat.cols(), gmat.rows());
  hrp::calcPseudoInverse(gmat, gmat_inv);
  wrench_ex = gmat_inv * gmat * wrench;
};

#undef PRINT_VECTOR

