/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef CLOSED_FORM_SOLVER_HPP
#define CLOSED_FORM_SOLVER_HPP
#include <Eigen/Dense>
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include "dgp/termcolor.hpp"
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <unsupported/Eigen/Polynomials>
#include <dynus/dynus_type.hpp>
#include "timer.hpp"

using namespace termcolor;
enum ConstraintType
{
  POSITION,
  VELOCITY,
  ACCELERATION,
  JERK
};

typedef timer::Timer MyTimer;

class ClosedFromSolver
{
public:

  ClosedFromSolver();

  // What we actually need for closed-form solution.
  void setX0(const state &data);
  void setXf(const state &data);
  void setPolytopes(std::vector<LinearConstraint3D> polytopes);
  void setT0(double t0);
  void setClosedFormInitialDt(double closed_form_initial_dt);
  bool findClosedFormSolution();
  void findClosedFormSolutionForEachAxis(double p0, double v0, double a0, double pf, double vf, double af, double T1, double T2, double T3, double &d1, double &c1, double &b1, double &a1, double &d2, double &c2, double &b2, double &a2, double &d3, double &c3, double &b3, double &a3);
  void fillGoalSetPoints();
  void getGoalSetpoints(std::vector<state> &goal_setpoints);
  void setClosedFormSolutionParams(int closed_form_time_allocation_adj_iter_max, double closed_form_initial_factor, double closed_form_factor_increment, double closed_form_factor_initial_decrement);
  void findDTForClosedForm(double factor);
  void checkDynamicViolation(bool &is_dyn_constraints_satisfied);
  void checkCollisionViolation(bool &is_collision_free_corridor_satisfied);
  void setDynamicConstraints();
  void getInitialAndFinalConditions(double &P0, double &V0, double &A0, double &Pf, double &Vf, double &Af, int axis);
  void setVerbose(bool verbose);
  void initializeGoalSetpoints();
  void setBounds(double max_values[3]);
  void setDC(double dc);
  void findIntervalIdxAndDt(double time_in_whole_traj, int &interval_idx, double &dt_interval);
  void findClosestIndexFromTime(const double t, int& index, const std::vector<double>& time);
  inline Eigen::Matrix<double, 3, 4> getPosControlPointsDouble(int t) const;
  void getPieceWisePol(PieceWisePol &pwp);
  void getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps);
  void getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps);

  inline double getPosDouble(int t, double tau, int ii) const;
  inline double getVelDouble(int t, double tau, int ii) const;
  inline double getAccelDouble(int t, double tau, int ii) const;
  inline double getJerkDouble(int t, double tau, int ii) const;

  inline double getAnDouble(int t, int ii) const;
  inline double getBnDouble(int t, int ii) const;
  inline double getCnDouble(int t, int ii) const;
  inline double getDnDouble(int t, int ii) const;

  inline std::vector<double> getCP0Double(int t) const;
  inline std::vector<double> getCP1Double(int t) const;
  inline std::vector<double> getCP2Double(int t) const;
  inline std::vector<double> getCP3Double(int t) const;

  // Get Minvo Control points given the interval as double
  inline Eigen::Matrix<double, 3, 4> getMinvoPosControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 3> getMinvoVelControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 2> getMinvoAccelControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 1> getMinvoJerkControlPointsDouble(int t) const;

  std::vector<state> goal_setpoints_;
  std::vector<double> dt_; // time step found by the solver
  double total_traj_time_;
  int N_ = 3;

protected:
  // parameters
  double xf_[3 * 3];
  double x0_[3 * 3];
  double t0_;
  double v_max_;
  double a_max_;
  double j_max_;
  double dc_;
  int current_polytopes_size_ = 3;
  int closed_form_time_allocation_adj_iter_max_;
  double closed_form_initial_dt_;
  double closed_form_initial_factor_;
  double closed_form_factor_;
  double closed_form_factor_increment_;
  double closed_form_factor_initial_decrement_;
  std::vector<LinearConstraint3D> polytopes_;

  // Basis converter
  BasisConverter basis_converter_;
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv_;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv_;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv_;

  // Flags
  bool debug_verbose_;

  // Variables
  std::vector<std::vector<double>> x_double_;
};
#endif