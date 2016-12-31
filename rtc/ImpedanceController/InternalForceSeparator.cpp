#include <hrpUtil/MatrixSolvers.h>
#include <iostream>
#include <iomanip>
#ifdef USE_QPOASES
#include <qpOASES.hpp>
using namespace qpOASES;
#endif
#include "InternalForceSeparator.h"

# define PRINT_VECTOR(vec) (vec.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")))

template <typename T>
void print_array(T arr, const unsigned int row, const unsigned int col) {
    std::ios::fmtflags flagsOrig = std::cerr.flags(); // save original format
    std::cerr.setf(std::ios::right, std::ios::adjustfield);
    std::cerr << std::setprecision(2);
    for (unsigned int i =0; i< row*col; i++) std::cerr << std::setw(8) << arr[i] << (((i+1) % col == 0) ? "\n" : ", ");
    std::cerr.flags(flagsOrig); // restore flags
};

InternalForceSeparator::InternalForceSeparator() : printp(false), debug_level(0), wrench_dim(6), use_moment(true) {};
void InternalForceSeparator::calcInternalForce(std::map<std::string, EndEffectorInfo> &ee_info) {
  /**
   * get ee_info and set ee_info[limb].xxx_ex, ee_info[limb].xxx_in,
   */
  hrp::dmatrix grasp_matrix = hrp::dmatrix::Zero(wrench_dim,wrench_dim*2);
  hrp::dvector wrench_abs=hrp::dvector::Zero(wrench_dim*2);
  hrp::dvector wrench_ref=hrp::dvector::Zero(wrench_dim*2);
  hrp::dvector wrench_abs_ex=hrp::dvector::Zero(wrench_dim*2);
  hrp::dvector wrench_abs_in=hrp::dvector::Zero(wrench_dim*2);
  hrp::dvector wrench_ref_ex=hrp::dvector::Zero(wrench_dim*2);
  hrp::dvector wrench_ref_in=hrp::dvector::Zero(wrench_dim*2);
  // calc wrenches
  calcGraspMatrix(grasp_matrix, ee_info);
  getAbsWrench(wrench_abs, ee_info);
  getRefWrench(wrench_ref, ee_info);
  if (use_qp) {
#ifdef USE_QPOASES
    calcExWrenchQPOASES(wrench_abs_ex, grasp_matrix, wrench_abs, ee_info);
    calcExWrenchQPOASES(wrench_ref_ex, grasp_matrix, wrench_ref, ee_info);
#else
    calcExWrench(wrench_abs_ex, grasp_matrix, wrench_abs, ee_info);
    calcExWrench(wrench_ref_ex, grasp_matrix, wrench_ref, ee_info);
#endif
  } else {
    calcExWrench(wrench_abs_ex, grasp_matrix, wrench_abs, ee_info);
    calcExWrench(wrench_ref_ex, grasp_matrix, wrench_ref, ee_info);
  }
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
    iter->second.ref_force_ex = wrench_ref_ex.segment<3>(0 + wrench_dim*limb_index);
    iter->second.ref_force_in = wrench_ref_in.segment<3>(0 + wrench_dim*limb_index);
    iter->second.abs_force_ex = wrench_abs_ex.segment<3>(0 + wrench_dim*limb_index);
    iter->second.abs_force_in = wrench_abs_in.segment<3>(0 + wrench_dim*limb_index);
    if (use_moment) {
      iter->second.ref_moment_ex = wrench_ref_ex.segment<3>(3 + wrench_dim*limb_index);
      iter->second.ref_moment_in = wrench_ref_in.segment<3>(3 + wrench_dim*limb_index);
      iter->second.abs_moment_ex = wrench_abs_ex.segment<3>(3 + wrench_dim*limb_index);
      iter->second.abs_moment_in = wrench_abs_in.segment<3>(3 + wrench_dim*limb_index);
    } else {
      iter->second.ref_moment_ex = iter->second.ref_moment;
      iter->second.abs_moment_ex = iter->second.abs_moment;
    }
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

const bool InternalForceSeparator::useMoment(bool use) {
  use_moment = use;
  if (use) wrench_dim = 6;
  else wrench_dim = 3;
  return static_cast<const bool>(use_moment);
};

const bool InternalForceSeparator::useMoment() {
  return static_cast<const bool>(use_moment);
};

const bool InternalForceSeparator::useQP(bool use) {
  use_qp = use && (! use_moment);
  return static_cast<const bool>(use_qp);
};

const bool InternalForceSeparator::useQP() {
  return static_cast<const bool>(use_qp);
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
  grasp_matrix = hrp::dmatrix::Zero(6,wrench_dim*2);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    grasp_matrix.block<3,3>(0,0+limb_index*wrench_dim) = hrp::dmatrix::Identity(3,3);
    if (use_moment) grasp_matrix.block<3,3>(3,3+limb_index*wrench_dim) = hrp::dmatrix::Identity(3,3);
    grasp_matrix.block<3,3>(3,0+limb_index*wrench_dim) = hrp::hat(iter->second.pos);
  }
};

void InternalForceSeparator::getAbsWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  wrench = hrp::dvector::Zero(wrench_dim*2);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    wrench.segment<3>(0+limb_index*wrench_dim) = iter->second.abs_force;
    if (use_moment) wrench.segment<3>(3+limb_index*wrench_dim) = iter->second.abs_moment;
  }
};

void InternalForceSeparator::getRefWrench(hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  wrench = hrp::dvector::Zero(wrench_dim*2);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
    size_t limb_index = std::distance(ee_info.begin(), iter);
    wrench.segment<3>(0+limb_index*wrench_dim) = iter->second.ref_force;
    if (use_moment) wrench.segment<3>(3+limb_index*wrench_dim) = iter->second.ref_moment;
  }
};

void InternalForceSeparator::calcExWrench(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  hrp::dmatrix gmat_inv = hrp::dmatrix::Zero(gmat.cols(), gmat.rows());
  hrp::calcPseudoInverse(gmat, gmat_inv);
  wrench_ex = gmat_inv * gmat * wrench;
};

#ifdef USE_QPOASES
void InternalForceSeparator::calcExWrenchQPOASES(hrp::dvector &wrench_ex, const hrp::dmatrix &gmat, const hrp::dvector &wrench, const std::map<std::string, EndEffectorInfo> &ee_info) {
  if (printp && debug_level > 0) std::cerr << "<====== calcExWrenchQPOASES" << std::endl;
  int nV = wrench_dim*2; // number of variable for qp
  // int nC = 5+2; // number of constraints (5: Equality, 2: Inequality)
  int nC = 6+2; // number of constraints (5: Equality, 2: Inequality)
  real_t* H = new real_t[nV*nV];
  real_t* g = new real_t[nV];
  real_t* lb = new real_t[nV];
  real_t* ub = new real_t[nV];
  real_t* A = new real_t[nC*nV];
  real_t* lbA = new real_t[nC];
  real_t* ubA = new real_t[nC];
  for (size_t i = 0; i < nV; i++) {
    for (size_t j = 0; j < nV; j++) {
      H[i*nV+j] = (i==j ? 1 : 0);
    }
    g[i] = 0;
    lb[i] = - 10000; // - todo: DBL_MAX maybe better
    ub[i] = + 10000; // + todo: DBL_MAX maybe better
  }
  hrp::dmatrix Ae = hrp::dmatrix::Zero(6,nV);
  hrp::dvector be = hrp::dvector::Zero(nV);
  Ae = gmat;
  be = gmat * wrench;
  // Ag h_{ex} >= bg^T
  hrp::dmatrix Ag = hrp::dmatrix::Zero(2,nV);
  hrp::dvector bl = hrp::dvector::Zero(2);
  for ( std::map<std::string, EndEffectorInfo>::const_iterator iter = ee_info.begin(); iter != ee_info.end(); iter++ ) {
      size_t limb_index = std::distance(ee_info.begin(), iter);
      Ag.block<1,3>(limb_index, 0+wrench_dim*limb_index) = iter->second.contact_force_dir;
  }
  if (printp && debug_level > 2) {
    std::cerr << "lb:" << std::endl; for (unsigned int i =0; i< nV; i++) std::cerr << lb[i] << ", "; std::cerr << std::endl;
    std::cerr << "ub:" << std::endl; for (unsigned int i =0; i< nV; i++) std::cerr << ub[i] << ", "; std::cerr << std::endl;
    std::cerr << "Ae:" << std::endl << Ae << std::endl;
    std::cerr << "be:" << std::endl << PRINT_VECTOR(be) << std::endl;
    std::cerr << "Ag:" << std::endl << Ag << std::endl;
    std::cerr << "bl:" << std::endl << PRINT_VECTOR(bl) << std::endl;
  }
  // convert matrix into qpoases real_t
  for (size_t i = 0; i < nC-2; i++) {
    for (size_t j = 0; j < nV; j++) {
      A[i*nV+j] = Ae(i,j);
    }
    lbA[i] = be(i);
    ubA[i] = be(i);
  }
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < nV; j++) {
      A[(i+nC-2)*nV+j] = Ag(i,j);
    }
    lbA[i+nC-2] = bl(i);
    ubA[i+nC-2] = + 10000; // + todo: DBL_MAX maybe better
  }
  if (printp && debug_level > 1) {
    std::cerr << "lbA:" << std::endl; print_array(lbA, 1, nC);
    std::cerr << "ubA:" << std::endl; print_array(ubA, 1, nC);
    std::cerr << "A:" << std::endl; print_array(A, nC, nV);
  }

  QProblem qprob( nV, nC, HST_IDENTITY );
  Options options;
  //options.enableFlippingBounds = BT_FALSE;
  options.initialStatusBounds = ST_INACTIVE;
  options.numRefinementSteps = 1;
  options.enableCholeskyRefactorisation = 1;
  //options.printLevel = PL_LOW;
  options.printLevel = PL_NONE;
  qprob.setOptions( options );
  /* Solve first QP. */
  int nWSR = 10;
  real_t cputime[1] = {0.001};
  qprob.init( 0, g, A, lb, ub, lbA, ubA, nWSR, cputime );
  real_t* xOpt = new real_t[nV];
  qprob.getPrimalSolution( xOpt );
  if (printp && debug_level > 0) { std::cerr << "xOpt: " << std::endl; print_array(xOpt, 1, nV); }
  for (unsigned int i=0; i < nV; i++) wrench_ex[i] = xOpt[i];
  if (printp && debug_level > 0) std::cerr << "wrench_ex: " << PRINT_VECTOR(wrench_ex) << std::endl;

  delete[] H;
  delete[] g;
  delete[] lb;
  delete[] ub;
  delete[] A;
  delete[] lbA;
  delete[] ubA;
  delete[] xOpt;
  if (printp && debug_level > 0) std::cerr << "======> calcExWrenchQPOASES" << std::endl;
};
#endif // calcExWrenchQPOASES

#undef PRINT_VECTOR
