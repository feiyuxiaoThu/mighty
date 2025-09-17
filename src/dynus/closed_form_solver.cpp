/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <dynus/closed_form_solver.hpp>
#include <chrono>
#include <unistd.h>

ClosedFromSolver::ClosedFromSolver()
{
  // Get basis converter (Q_{MINVO} = M_{BE2MV} * Q_{BEZIER})
  // Get std version of the basis converter
  A_pos_mv_rest_inv_ = basis_converter_.A_pos_mv_rest_inv;
  A_vel_mv_rest_inv_ = basis_converter_.A_vel_mv_rest_inv;
  A_accel_mv_rest_inv_ = basis_converter_.A_accel_mv_rest_inv;
}

void ClosedFromSolver::setVerbose(bool verbose)
{
  debug_verbose_ = verbose;
}

void ClosedFromSolver::setClosedFormSolutionParams(int closed_form_time_allocation_adj_iter_max, double closed_form_initial_factor, double closed_form_factor_increment, double closed_form_factor_initial_decrement)
{
  closed_form_time_allocation_adj_iter_max_ = closed_form_time_allocation_adj_iter_max;
  closed_form_initial_factor_ = closed_form_initial_factor;
  closed_form_factor_ = closed_form_initial_factor_;
  closed_form_factor_increment_ = closed_form_factor_increment;
  closed_form_factor_initial_decrement_ = closed_form_initial_factor_ - closed_form_factor_increment_;
}

void ClosedFromSolver::setT0(double t0)
{
  t0_ = t0;
}

// Given the time in the whole trajectory, find the interval in which the time is located and the dt within the interval
void ClosedFromSolver::findIntervalIdxAndDt(double time_in_whole_traj, int &interval_idx, double &dt_interval)
{

  // Find the interval and the time within the interval
  double dt_sum = 0.0;
  for (int interval = 0; interval < N_; interval++)
  {

    dt_sum += dt_[interval];

    if (time_in_whole_traj < dt_sum)
    {
      interval_idx = interval;
      break;
    }
  }

  if (interval_idx == 0)
  {
    dt_interval = time_in_whole_traj;
  }
  else
  {
    dt_interval = time_in_whole_traj - (dt_sum - dt_[interval_idx]);
  }
}

void ClosedFromSolver::findClosestIndexFromTime(const double t, int &index, const std::vector<double> &time)
{
  // Find the closest index from the time vector
  double min_diff = std::numeric_limits<double>::max();
  for (int i = 0; i < time.size(); i++)
  {
    double diff = std::abs(time[i] - t);
    if (diff < min_diff)
    {
      min_diff = diff;
      index = i;
    }
  }
}

void ClosedFromSolver::checkDynamicViolation(bool &is_dyn_constraints_satisfied)
{

  // Check if the time allocation satisfies the dynamic constraints
  is_dyn_constraints_satisfied = true;
  for (int segment = 0; segment < N_; segment++)
  {
    /// velocity
    Eigen::Matrix<double, 3, 3> Vn_prime = getMinvoVelControlPointsDouble(segment);

    // for control points (colunmns of Vn_prime)
    for (int i = 0; i < 3; i++)
    {
      if (Vn_prime.col(i)[0] < -v_max_ || Vn_prime.col(i)[0] > v_max_ ||
          Vn_prime.col(i)[1] < -v_max_ || Vn_prime.col(i)[1] > v_max_ ||
          Vn_prime.col(i)[2] < -v_max_ || Vn_prime.col(i)[2] > v_max_)
      {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// acceleration
    Eigen::Matrix<double, 3, 2> Vn_double_prime = getMinvoAccelControlPointsDouble(segment);

    // for control points (colunmns of Vn_double_prime)
    for (int i = 0; i < 2; i++)
    {
      if (Vn_double_prime.col(i)[0] < -a_max_ || Vn_double_prime.col(i)[0] > a_max_ ||
          Vn_double_prime.col(i)[1] < -a_max_ || Vn_double_prime.col(i)[1] > a_max_ ||
          Vn_double_prime.col(i)[2] < -a_max_ || Vn_double_prime.col(i)[2] > a_max_)
      {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// jerk
    Eigen::Matrix<double, 3, 1> Vn_triple_prime = getMinvoJerkControlPointsDouble(segment);

    // for control points (colunmns of Vn_triple_prime)
    if (Vn_triple_prime.col(0)[0] < -j_max_ || Vn_triple_prime.col(0)[0] > j_max_ ||
        Vn_triple_prime.col(0)[1] < -j_max_ || Vn_triple_prime.col(0)[1] > j_max_ ||
        Vn_triple_prime.col(0)[2] < -j_max_ || Vn_triple_prime.col(0)[2] > j_max_)
    {
      is_dyn_constraints_satisfied = false;
      break;
    }

  } // end for segment
}

void ClosedFromSolver::checkCollisionViolation(bool &is_collision_free_corridor_satisfied)
{

  is_collision_free_corridor_satisfied = true;
  for (int t = 0; t < N_; t++)
  {                                 // Loop through each segment
    bool segment_satisfied = false; // Will be true if at least one polytope contains this segment

    // Retrieve the closed-form MINVO position control points for segment t.
    // Here we assume the function returns a 3x4 matrix (each column is a control point)
    Eigen::Matrix<double, 3, 4> cps = getMinvoPosControlPointsDouble(t);

    // Check every polytope until one is found that contains the segment
    // for (const auto &polytope : polytopes_)
    for (int polytope_idx = 0; polytope_idx < polytopes_.size(); polytope_idx++)
    {

      // Assume this polytope works until proven otherwise
      bool polytope_satisfied = true;

      // Check every control point in the segment.
      // (You could also check the convex hull of cps, but here we simply test all cps.)
      for (int j = 0; j < cps.cols(); j++)
      {
        if (!polytopes_[polytope_idx].inside(cps.col(j)))
        {
          polytope_satisfied = false;
          break; // This polytope fails for this constraint
        }
      }

      // If this polytope satisfied all constraints for all control points, mark segment as satisfied.
      if (polytope_satisfied)
      {
        segment_satisfied = true;
        break; // No need to check other polytopes for this segment
      }
    } // end polytope loop

    // If the segment was not contained in any polytope, flag the overall corridor as unsatisfied.
    if (!segment_satisfied)
    {
      is_collision_free_corridor_satisfied = false;
      break; // Early exit: one segment failed, so overall constraint is not satisfied.
    }
  }
}

/**
 * @brief Find the closest index from the time vector
 */
bool ClosedFromSolver::findClosedFormSolution()
{

  // Find the closed-form solution for each axis
  std::vector<double> p0, v0, a0, pf, vf, af;
  for (int axis = 0; axis < 3; axis++)
  {
    p0.push_back(x0_[axis]);
    v0.push_back(x0_[axis + 3]);
    a0.push_back(x0_[axis + 6]);

    pf.push_back(xf_[axis]);
    vf.push_back(xf_[axis + 3]);
    af.push_back(xf_[axis + 6]);
  }

  // Initialize the variables
  double d0x, c0x, b0x, a0x, d1x, c1x, b1x, a1x, d2x, c2x, b2x, a2x;
  double d0y, c0y, b0y, a0y, d1y, c1y, b1y, a1y, d2y, c2y, b2y, a2y;
  double d0z, c0z, b0z, a0z, d1z, c1z, b1z, a1z, d2z, c2z, b2z, a2z;

  // We adjust the time allocation to satisfy the dynamic constraints

  // we first go down a bit to test limits of time allocation
  // closed_form_factor_ -= closed_form_factor_initial_decrement_;
  closed_form_factor_ = closed_form_initial_factor_;

  bool is_dyn_constraints_satisfied = false;
  bool is_collision_free_corridor_satisfied = false;

  for (int time_allocation_adj_iter = 0; time_allocation_adj_iter < closed_form_time_allocation_adj_iter_max_; time_allocation_adj_iter++)
  {

    findDTForClosedForm(closed_form_factor_);

    // Find the time allocation
    double T1 = dt_[0];
    double T2 = dt_[1];
    double T3 = dt_[2];

    // Find the closed-form solution for each axis
    findClosedFormSolutionForEachAxis(p0[0], v0[0], a0[0], pf[0], vf[0], af[0], T1, T2, T3, d0x, c0x, b0x, a0x, d1x, c1x, b1x, a1x, d2x, c2x, b2x, a2x);
    findClosedFormSolutionForEachAxis(p0[1], v0[1], a0[1], pf[1], vf[1], af[1], T1, T2, T3, d0y, c0y, b0y, a0y, d1y, c1y, b1y, a1y, d2y, c2y, b2y, a2y);
    findClosedFormSolutionForEachAxis(p0[2], v0[2], a0[2], pf[2], vf[2], af[2], T1, T2, T3, d0z, c0z, b0z, a0z, d1z, c1z, b1z, a1z, d2z, c2z, b2z, a2z);

    // Set the solution
    x_double_.clear();
    x_double_.push_back({a0x, b0x, c0x, d0x, a1x, b1x, c1x, d1x, a2x, b2x, c2x, d2x});
    x_double_.push_back({a0y, b0y, c0y, d0y, a1y, b1y, c1y, d1y, a2y, b2y, c2y, d2y});
    x_double_.push_back({a0z, b0z, c0z, d0z, a1z, b1z, c1z, d1z, a2z, b2z, c2z, d2z});

    // Check if x_double_ has any NaN values
    bool has_nan = false;
    for (int i = 0; i < x_double_.size(); i++) // for each axis
    {
      for (int j = 0; j < x_double_[i].size(); j++) // for each coefficient
      {
        if (std::isnan(x_double_[i][j]))
        {
          std::cout << "x_double_ has NaN values" << std::endl;
          has_nan = true;
          break;
        }
      }
      if (has_nan)
        break;
    }

    if (has_nan)
      break;

    // Check if the dynamic constraints and the collision-free corridor constraints are satisfied
    checkDynamicViolation(is_dyn_constraints_satisfied);
    checkCollisionViolation(is_collision_free_corridor_satisfied);

    // If both the dynamic constraints and the collision-free corridor constraints are satisfied, break the loop
    if (is_dyn_constraints_satisfied && is_collision_free_corridor_satisfied)
    {
      break;
    }

    // If the constraints are not satisfied, adjust the time allocation for the next iteration
    closed_form_factor_ += closed_form_factor_increment_;
  }

  // If after the maximum number of iterations, the dynamic constraints are still not satisfied, return false
  if (!is_dyn_constraints_satisfied)
  {
    if (debug_verbose_)
      std::cout << "Dynamic constraints are not satisfied" << std::endl;
    return false;
  }

  // If after the maximum number of iterations, the collision-free corridor constraints are still not satisfied, return false
  if (!is_collision_free_corridor_satisfied)
  {
    if (debug_verbose_)
      std::cout << "Collision-free corridor constraints are not satisfied" << std::endl;
    return false;
  }

  // Success
  initializeGoalSetpoints();
  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Find a closed-form solution for each axis.
 */
void ClosedFromSolver::findClosedFormSolutionForEachAxis(double p0, double v0, double a0, double pf, double vf, double af, double T1, double T2, double T3, double &d1, double &c1, double &b1, double &a1, double &d2, double &c2, double &b2, double &a2, double &d3, double &c3, double &b3, double &a3)
{
  // Segment 1:
  d1 = p0;
  c1 = v0;
  b1 = (1.0 / 2.0) * a0;
  a1 = (1.0 / 6.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (T1 * (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3));
  // Segment 2:
  d2 = ((1.0 / 3.0) * pow(T1, 3) * T2 * a0 + (1.0 / 6.0) * pow(T1, 3) * T3 * a0 + (1.0 / 3.0) * pow(T1, 2) * pow(T2, 2) * a0 + (1.0 / 3.0) * pow(T1, 2) * T2 * T3 * a0 + (1.0 / 6.0) * pow(T1, 2) * T2 * T3 * af + (4.0 / 3.0) * pow(T1, 2) * T2 * v0 - 1.0 / 3.0 * pow(T1, 2) * T2 * vf + (1.0 / 6.0) * pow(T1, 2) * pow(T3, 2) * af + (2.0 / 3.0) * pow(T1, 2) * T3 * v0 - 2.0 / 3.0 * pow(T1, 2) * T3 * vf + pow(T1, 2) * pf + T1 * pow(T2, 2) * v0 + T1 * T2 * T3 * v0 + 2 * T1 * T2 * p0 + T1 * T3 * p0 + pow(T2, 2) * p0 + T2 * T3 * p0) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  c2 = (-1.0 / 2.0 * pow(T1, 3) * a0 - 2 * pow(T1, 2) * v0 + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 + (1.0 / 2.0) * T1 * T2 * T3 * af - T1 * T2 * vf + (1.0 / 2.0) * T1 * pow(T3, 2) * af - 2 * T1 * T3 * vf - 3 * T1 * p0 + 3 * T1 * pf + pow(T2, 2) * v0 + T2 * T3 * v0) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  b2 = (1.0 / 2.0) * a0 + (1.0 / 2.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  a2 = ((1.0 / 6.0) * pow(T1, 3) * a0 + (2.0 / 3.0) * pow(T1, 2) * T2 * a0 + (1.0 / 3.0) * pow(T1, 2) * T3 * a0 - 1.0 / 6.0 * pow(T1, 2) * T3 * af + (2.0 / 3.0) * pow(T1, 2) * v0 + (1.0 / 3.0) * pow(T1, 2) * vf + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + 2 * T1 * T2 * v0 + T1 * T2 * vf + (1.0 / 6.0) * T1 * pow(T3, 2) * a0 - 1.0 / 3.0 * T1 * pow(T3, 2) * af + T1 * T3 * v0 + T1 * T3 * vf + T1 * p0 - T1 * pf - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * v0 + pow(T2, 2) * vf - 2.0 / 3.0 * T2 * pow(T3, 2) * af + T2 * T3 * v0 + 2 * T2 * T3 * vf + 2 * T2 * p0 - 2 * T2 * pf - 1.0 / 6.0 * pow(T3, 3) * af + (1.0 / 3.0) * pow(T3, 2) * v0 + (2.0 / 3.0) * pow(T3, 2) * vf + T3 * p0 - T3 * pf) / (T2 * (pow(T1, 2) * T2 + pow(T1, 2) * T3 + 2 * T1 * pow(T2, 2) + 3 * T1 * T2 * T3 + T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)));
  // Segment 3:
  d3 = ((1.0 / 6.0) * pow(T1, 2) * pow(T3, 2) * a0 + (1.0 / 6.0) * T1 * T2 * pow(T3, 2) * a0 + (1.0 / 3.0) * T1 * T2 * pow(T3, 2) * af - T1 * T2 * T3 * vf + T1 * T2 * pf + (1.0 / 6.0) * T1 * pow(T3, 3) * af + (2.0 / 3.0) * T1 * pow(T3, 2) * v0 - 2.0 / 3.0 * T1 * pow(T3, 2) * vf + T1 * T3 * pf + (1.0 / 3.0) * pow(T2, 2) * pow(T3, 2) * af - pow(T2, 2) * T3 * vf + pow(T2, 2) * pf + (1.0 / 3.0) * T2 * pow(T3, 3) * af + (1.0 / 3.0) * T2 * pow(T3, 2) * v0 - 4.0 / 3.0 * T2 * pow(T3, 2) * vf + 2 * T2 * T3 * pf + pow(T3, 2) * p0) / (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2));
  c3 = (-1.0 / 2.0 * pow(T1, 2) * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + T1 * T2 * vf - 2 * T1 * T3 * v0 - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * vf - T2 * T3 * v0 + (1.0 / 2.0) * pow(T3, 3) * af - 2 * pow(T3, 2) * vf - 3 * T3 * p0 + 3 * T3 * pf) / (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2));
  b3 = (1.0 / 2.0) * a0 + 3 * ((1.0 / 6.0) * pow(T1, 3) * a0 + (2.0 / 3.0) * pow(T1, 2) * T2 * a0 + (1.0 / 3.0) * pow(T1, 2) * T3 * a0 - 1.0 / 6.0 * pow(T1, 2) * T3 * af + (2.0 / 3.0) * pow(T1, 2) * v0 + (1.0 / 3.0) * pow(T1, 2) * vf + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + 2 * T1 * T2 * v0 + T1 * T2 * vf + (1.0 / 6.0) * T1 * pow(T3, 2) * a0 - 1.0 / 3.0 * T1 * pow(T3, 2) * af + T1 * T3 * v0 + T1 * T3 * vf + T1 * p0 - T1 * pf - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * v0 + pow(T2, 2) * vf - 2.0 / 3.0 * T2 * pow(T3, 2) * af + T2 * T3 * v0 + 2 * T2 * T3 * vf + 2 * T2 * p0 - 2 * T2 * pf - 1.0 / 6.0 * pow(T3, 3) * af + (1.0 / 3.0) * pow(T3, 2) * v0 + (2.0 / 3.0) * pow(T3, 2) * vf + T3 * p0 - T3 * pf) / (pow(T1, 2) * T2 + pow(T1, 2) * T3 + 2 * T1 * pow(T2, 2) + 3 * T1 * T2 * T3 + T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)) + (1.0 / 2.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  a3 = (1.0 / 6.0) * (-pow(T1, 2) * a0 - T1 * T2 * a0 + T1 * T2 * af + 2 * T1 * T3 * af - 4 * T1 * v0 - 2 * T1 * vf + pow(T2, 2) * af + 4 * T2 * T3 * af - 2 * T2 * v0 - 4 * T2 * vf + 3 * pow(T3, 2) * af - 6 * T3 * vf - 6 * p0 + 6 * pf) / (T3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)));
}

void ClosedFromSolver::fillGoalSetPoints()
{

  const int num_goal_setpoints = goal_setpoints_.size();

  // Precompute timestamps
  std::vector<double> timestamps(num_goal_setpoints);
  for (int i = 0; i < num_goal_setpoints; i++)
    timestamps[i] = (i + 1) * dc_;

  #pragma omp parallel for
  for (int i = 0; i < num_goal_setpoints; i++)
  {
    // Get the timestamp
    double t = timestamps[i];

    // Find interval index and dt_interval (assumed thread-safe)
    int interval_idx = 0; // initialize to 0
    double dt_interval = 0; // initialize to 0
    findIntervalIdxAndDt(t, interval_idx, dt_interval);

    // Compute positions
    double posx = getPosDouble(interval_idx, dt_interval, 0);
    double posy = getPosDouble(interval_idx, dt_interval, 1);
    double posz = getPosDouble(interval_idx, dt_interval, 2);

    // Compute velocities
    double velx = getVelDouble(interval_idx, dt_interval, 0);
    double vely = getVelDouble(interval_idx, dt_interval, 1);
    double velz = getVelDouble(interval_idx, dt_interval, 2);

    // Compute accelerations
    double accelx = getAccelDouble(interval_idx, dt_interval, 0);
    double accely = getAccelDouble(interval_idx, dt_interval, 1);
    double accelz = getAccelDouble(interval_idx, dt_interval, 2);

    // Compute jerks
    double jerkx = getJerkDouble(interval_idx, dt_interval, 0);
    double jerky = getJerkDouble(interval_idx, dt_interval, 1);
    double jerkz = getJerkDouble(interval_idx, dt_interval, 2);

    // Set the state (assuming thread-safe)
    state state_i;
    state_i.setTimeStamp(t0_ + t);
    state_i.setPos(posx, posy, posz);
    state_i.setVel(velx, vely, velz);
    state_i.setAccel(accelx, accely, accelz);
    state_i.setJerk(jerkx, jerky, jerkz);

    // Set the state
    goal_setpoints_[i] = state_i;
  }

  // Ensure the final input is explicitly zeroed (serially after parallel loop)
  goal_setpoints_[num_goal_setpoints - 1].vel = Eigen::Vector3d::Zero().transpose();
  goal_setpoints_[num_goal_setpoints - 1].accel = Eigen::Vector3d::Zero().transpose();
  goal_setpoints_[num_goal_setpoints - 1].jerk = Eigen::Vector3d::Zero().transpose();

}

void ClosedFromSolver::getGoalSetpoints(std::vector<state> &goal_setpoints)
{
  goal_setpoints = goal_setpoints_;
}

void ClosedFromSolver::setPolytopes(std::vector<LinearConstraint3D> polytopes)
{
  // Set polytopes
  polytopes_ = polytopes;
  // Set polytopes size
  current_polytopes_size_ = polytopes.size();
}

void ClosedFromSolver::setDC(double dc)
{
  dc_ = dc;
}

void ClosedFromSolver::setX0(const state &data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  x0_[0] = data.pos.x();
  x0_[1] = data.pos.y();
  x0_[2] = data.pos.z();
  x0_[3] = data.vel.x();
  x0_[4] = data.vel.y();
  x0_[5] = data.vel.z();
  x0_[6] = data.accel.x();
  x0_[7] = data.accel.y();
  x0_[8] = data.accel.z();
}

void ClosedFromSolver::setXf(const state &data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  xf_[0] = data.pos.x();
  xf_[1] = data.pos.y();
  xf_[2] = data.pos.z();
  xf_[3] = data.vel.x();
  xf_[4] = data.vel.y();
  xf_[5] = data.vel.z();
  xf_[6] = data.accel.x();
  xf_[7] = data.accel.y();
  xf_[8] = data.accel.z();
}

void ClosedFromSolver::initializeGoalSetpoints()
{
  int size = static_cast<int>(total_traj_time_ / dc_);
  size = std::max(size, 2);
  goal_setpoints_.resize(size);
}

void ClosedFromSolver::setBounds(double max_values[3])
{
  v_max_ = max_values[0];
  a_max_ = max_values[1];
  j_max_ = max_values[2];
}

void ClosedFromSolver::setClosedFormInitialDt(double closed_form_initial_dt)
{
  closed_form_initial_dt_ = closed_form_initial_dt;
}

void ClosedFromSolver::findDTForClosedForm(double factor)
{

  // Clear dt_
  dt_.clear();

  // Compute the dt for the closed form solution
  for (int i = 0; i < N_; i++)
  {
    dt_.push_back(factor * closed_form_initial_dt_);
  }

  // Compute total_traj_time_
  total_traj_time_ = std::accumulate(dt_.begin(), dt_.end(), 0.0);
}

void ClosedFromSolver::getPieceWisePol(PieceWisePol &pwp)
{
  // Reset the piecewise polynomial
  pwp.clear();

  // Get the times of the piecewise polynomial
  double t = 0.0;
  for (int i = 0; i < N_ + 1; i++)
  {

    if (i == 0)
    {
      pwp.times.push_back(t0_);
      continue;
    }

    t = t + dt_[i - 1];

    // Add the time to the piecewise polynomial
    pwp.times.push_back(t0_ + t);
  }

  // Get the coefficients of the piecewise polynomial
  for (int i = 0; i < N_; i++)
  {

    // Initialize the coefficients
    Eigen::Matrix<double, 4, 1> coeff_x_i;
    Eigen::Matrix<double, 4, 1> coeff_y_i;
    Eigen::Matrix<double, 4, 1> coeff_z_i;

    // a, b, c, d
    for (int j = 0; j < 4; j++)
    {
      coeff_x_i(j) = x_double_[0][i * 4 + j];
      coeff_y_i(j) = x_double_[1][i * 4 + j];
      coeff_z_i(j) = x_double_[2][i * 4 + j];
    }

    // Add the coefficients to the piecewise polynomial
    pwp.coeff_x.push_back(coeff_x_i);
    pwp.coeff_y.push_back(coeff_y_i);
    pwp.coeff_z.push_back(coeff_z_i);
  }
}

void ClosedFromSolver::getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++)
  {
    cps.push_back(getMinvoPosControlPointsDouble(t));
  }
}

void ClosedFromSolver::getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++)
  {
    cps.push_back(getPosControlPointsDouble(t));
  }
}

// -----------------------------------------------------------------------------

inline double ClosedFromSolver::getPosDouble(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return x_double_[axis][base_index] * tau * tau * tau + x_double_[axis][base_index + 1] * tau * tau + x_double_[axis][base_index + 2] * tau + x_double_[axis][base_index + 3];
}

inline double ClosedFromSolver::getVelDouble(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  // For the numeric (double) version we assume the solved (numeric) values are stored in x_double_
  return 3 * x_double_[axis][base_index] * tau * tau +
         2 * x_double_[axis][base_index + 1] * tau +
         x_double_[axis][base_index + 2];
}

inline double ClosedFromSolver::getAccelDouble(int interval,
                                               double tau,
                                               int axis) const
{
  int base = 4 * interval;
  double Ts = dt_[interval];

  // second derivative wrt u = 6 a₃ u + 2 a₂
  double d2P_du2 =
      6.0 * x_double_[axis][base] * tau + 2.0 * x_double_[axis][base + 1];

  // d²u/dt² = 1/Ts²
  return d2P_du2 / (Ts * Ts);
}

inline double ClosedFromSolver::getJerkDouble(int interval,
                                              double /*tau*/,
                                              int axis) const
{
  int base = 4 * interval;
  double Ts = dt_[interval];

  // third derivative wrt u = 6 a₃
  double d3P_du3 = 6.0 * x_double_[axis][base];

  // d³u/dt³ = 1/Ts³
  return d3P_du3 / (Ts * Ts * Ts);
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
inline double ClosedFromSolver::getAnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline double ClosedFromSolver::getBnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline double ClosedFromSolver::getCnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 2] * dt_[interval];
}

inline double ClosedFromSolver::getDnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 3];
}

inline std::vector<double> ClosedFromSolver::getCP0Double(int interval) const
{
  std::vector<double> cp = {getPosDouble(interval, 0, 0), getPosDouble(interval, 0, 1), getPosDouble(interval, 0, 2)};
  return cp;
}

inline std::vector<double> ClosedFromSolver::getCP1Double(int interval) const
{
  double cpx = (getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> ClosedFromSolver::getCP2Double(int interval) const
{
  double cpx = (getBnDouble(interval, 0) + 2 * getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getBnDouble(interval, 1) + 2 * getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getBnDouble(interval, 2) + 2 * getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> ClosedFromSolver::getCP3Double(int interval) const
{
  std::vector<double> cp = {getPosDouble(interval, dt_[interval], 0), getPosDouble(interval, dt_[interval], 1), getPosDouble(interval, dt_[interval], 2)};
  return cp;
}

inline Eigen::Matrix<double, 3, 4> ClosedFromSolver::getMinvoPosControlPointsDouble(int interval) const
{
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  Pn << getAnDouble(interval, 0), getBnDouble(interval, 0), getCnDouble(interval, 0), getDnDouble(interval, 0),
      getAnDouble(interval, 1), getBnDouble(interval, 1), getCnDouble(interval, 1), getDnDouble(interval, 1),
      getAnDouble(interval, 2), getBnDouble(interval, 2), getCnDouble(interval, 2), getDnDouble(interval, 2);

  // Convert from coefficients of polynomials to MINVO control points using the precomputed conversion matrix.
  Eigen::Matrix<double, 3, 4> Vn = Pn * A_pos_mv_rest_inv_;
  return Vn;
}

inline Eigen::Matrix<double, 3, 3> ClosedFromSolver::getMinvoVelControlPointsDouble(int interval) const
{
  // Retrieve the duration for segment t.
  double deltaT = dt_[interval];

  // Compute the derivative with respect to the normalized variable u:
  // dp/du = 3*Aₙ*u² + 2*Bₙ*u + Cₙ.
  // Then, using the chain rule, dp/dt = (dp/du) (du/dt).
  // du / dt = 1 / deltaT.
  Eigen::Matrix<double, 3, 3> Pn_prime;
  Pn_prime << (3 * getAnDouble(interval, 0)) / deltaT, (2 * getBnDouble(interval, 0)) / deltaT, getCnDouble(interval, 0) / deltaT,
      (3 * getAnDouble(interval, 1)) / deltaT, (2 * getBnDouble(interval, 1)) / deltaT, getCnDouble(interval, 1) / deltaT,
      (3 * getAnDouble(interval, 2)) / deltaT, (2 * getBnDouble(interval, 2)) / deltaT, getCnDouble(interval, 2) / deltaT;

  // Convert the normalized derivative to MINVO velocity control points.
  Eigen::Matrix<double, 3, 3> Vn_prime = Pn_prime * A_vel_mv_rest_inv_;
  return Vn_prime;
}

inline Eigen::Matrix<double, 3, 2> ClosedFromSolver::getMinvoAccelControlPointsDouble(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT2 = deltaT * deltaT;

  // Compute the second derivative with respect to u:
  // d²p/du² = 6*Aₙ*u + 2*Bₙ.
  // Then, using the chain rule, d²p/dt² = (d²p/du²) (du/dt)².
  Eigen::Matrix<double, 3, 2> Pn_double_prime;
  Pn_double_prime << (6 * getAnDouble(interval, 0)) / deltaT2, (2 * getBnDouble(interval, 0)) / deltaT2,
      (6 * getAnDouble(interval, 1)) / deltaT2, (2 * getBnDouble(interval, 1)) / deltaT2,
      (6 * getAnDouble(interval, 2)) / deltaT2, (2 * getBnDouble(interval, 2)) / deltaT2;

  // Convert the normalized second derivative to MINVO acceleration control points.
  Eigen::Matrix<double, 3, 2> Vn_double_prime = Pn_double_prime * A_accel_mv_rest_inv_;
  return Vn_double_prime;
}

inline Eigen::Matrix<double, 3, 1> ClosedFromSolver::getMinvoJerkControlPointsDouble(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT3 = deltaT * deltaT * deltaT;

  // Compute the third derivative with respect to u:
  // d³p/du³ = 6*Aₙ.
  // Then, using the chain rule, d³p/dt³ = (d³p/du³) (du/dt)³.
  Eigen::Matrix<double, 3, 1> Pn_triple_prime;
  Pn_triple_prime << (6 * getAnDouble(interval, 0)) / deltaT3,
      (6 * getAnDouble(interval, 1)) / deltaT3,
      (6 * getAnDouble(interval, 2)) / deltaT3;

  // Here we assume that no further conversion is required for jerk.
  Eigen::Matrix<double, 3, 1> Vn_triple_prime = Pn_triple_prime;
  return Vn_triple_prime;
}

inline Eigen::Matrix<double, 3, 4> ClosedFromSolver::getPosControlPointsDouble(int interval) const
{
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  std::vector<double> cp0 = getCP0Double(interval);
  std::vector<double> cp1 = getCP1Double(interval);
  std::vector<double> cp2 = getCP2Double(interval);
  std::vector<double> cp3 = getCP3Double(interval);
  Pn << cp0[0], cp1[0], cp2[0], cp3[0],
      cp0[1], cp1[1], cp2[1], cp3[1],
      cp0[2], cp1[2], cp2[2], cp3[2];
  return Pn;
}