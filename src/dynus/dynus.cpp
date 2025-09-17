/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "dynus/dynus.hpp"

using namespace dynus;
using namespace termcolor;

typedef timer::Timer MyTimer;

// ----------------------------------------------------------------------------

/**
 * @brief Constructor for DYNUS.
 * @param parameters par: Input configuration parameters.
 */
DYNUS::DYNUS(parameters par) : par_(par)
{

  // Set up dgp_manager
  dgp_manager_.setParameters(par_);

  // Set up the planner parameters (TODO: move to parameters)
  planner_params_.verbose = false;                                 // enable verbose output
  planner_params_.V_nom = par_.v_nom;                              // nominal velocity
  planner_params_.V_max = par_.v_max;                              // max velocity
  planner_params_.A_max = par_.a_max;                              // max acceleration
  planner_params_.J_max = par_.j_max;                              // max jerk
  planner_params_.num_perturbation = par_.num_perturbation_for_ig; // number of perturbations for initial guesses
  planner_params_.r_max = par_.r_max_for_ig;                       // perturbation radius for initial guesses
  planner_params_.time_weight = par_.time_weight;                  // weight for time cost
  planner_params_.dyn_weight = par_.dynamic_weight;
  planner_params_.stat_weight = par_.stat_weight;
  planner_params_.jerk_weight = par_.jerk_weight;
  planner_params_.dyn_constr_vel_weight = par_.dyn_constr_vel_weight;
  planner_params_.dyn_constr_acc_weight = par_.dyn_constr_acc_weight;
  planner_params_.dyn_constr_jerk_weight = par_.dyn_constr_jerk_weight;
  planner_params_.dyn_constr_bodyrate_weight = par_.dyn_constr_bodyrate_weight;
  planner_params_.dyn_constr_tilt_weight = par_.dyn_constr_tilt_weight;
  planner_params_.dyn_constr_thrust_weight = par_.dyn_constr_thrust_weight;
  planner_params_.num_dyn_obst_samples = par_.num_dyn_obst_samples; // Number of dynamic obstacle samples
  planner_params_.Co = par_.planner_Co;                             // for static obstacle avoidance
  planner_params_.Cw = par_.planner_Cw;                             // for dynamic obstacle avoidance
  planner_params_.BIG = 1e8;
  planner_params_.dc = par_.dc;                                             // descretiation constant
  planner_params_.second_to_last_vel_scale = par_.second_to_last_vel_scale; // scale for the second to last velocity vector
  planner_params_.init_turn_bf = par_.init_turn_bf;

  // Set up the L-BFGS parameters
  lbfgs_params_.mem_size = 256;
  lbfgs_params_.min_step = 1.0e-32;
  lbfgs_params_.f_dec_coeff = par_.f_dec_coeff;         // allow larger Armijo steps
  lbfgs_params_.cautious_factor = par_.cautious_factor; // always accept BFGS update
  lbfgs_params_.past = par_.past;
  lbfgs_params_.max_linesearch = par_.max_linesearch; // fewer backtracking tries
  lbfgs_params_.max_iterations = par_.max_iterations; // allow more iterations
  lbfgs_params_.g_epsilon = par_.g_epsilon;
  lbfgs_params_.delta = par_.delta; // stop once f-improvement is minimal

  // Set up unconstrained optimization solver for whole trajectory
  whole_traj_solver_ptr_ = std::make_shared<lbfgs::SolverLBFGS>();
  whole_traj_solver_ptr_->initializeSolver(planner_params_);

  // Set up closed-form solver for safe trajectory
  for (int i = 0; i < par_.num_safe_paths; i++)
  {
    safe_traj_solver_ptrs_.push_back(std::make_shared<ClosedFromSolver>());
    initializeSolver(safe_traj_solver_ptrs_[i]); // Use closed-form -> N = 3
  }

  // Set up closed-form solver for contingency trajectory
  contingency_traj_solver_ptr_ = std::make_shared<ClosedFromSolver>();
  initializeSolver(contingency_traj_solver_ptr_); // Use closed-form -> N = 3

  // Set up basis converter
  BasisConverter basis_converter;
  A_rest_pos_basis_ = basis_converter.getArestMinvo(); // Use Minvo basis
  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

  // Parameters
  v_max_3d_ = Eigen::Vector3d(par_.v_max, par_.v_max, par_.v_max);
  v_max_ = par_.v_max;
  a_max_3d_ = Eigen::Vector3d(par_.a_max, par_.a_max, par_.a_max);
  j_max_3d_ = Eigen::Vector3d(par_.j_max, par_.j_max, par_.j_max);

  // Initialize the state
  changeDroneStatus(DroneStatus::GOAL_REACHED);

  // Initialize frontier flag
  use_frontiers_as_G_ = par_.use_frontiers;

  // Initialize the map size
  wdx_ = par_.initial_wdx;
  wdy_ = par_.initial_wdy;
  wdz_ = par_.initial_wdz;

  // Map resolution
  map_res_ = par_.res;
}

// ----------------------------------------------------------------------------

/**
 * @brief Initializes the Gurobi solver.
 * @param int num_N: Number of segments for the optimization.
 */
void DYNUS::initializeSolver(const std::shared_ptr<ClosedFromSolver> &traj_solver_ptr_)
{
  traj_solver_ptr_->setVerbose(par_.closed_form_traj_verbose);
  traj_solver_ptr_->setClosedFormSolutionParams(par_.closed_form_time_allocation_adj_iter_max, par_.closed_form_initial_factor, par_.closed_form_factor_increment, par_.closed_form_factor_initial_decrement);
  traj_solver_ptr_->setDC(par_.dc);
  double max_values[3] = {par_.v_max, par_.a_max, par_.j_max};
  traj_solver_ptr_->setBounds(max_values);
}

// ----------------------------------------------------------------------------

/**
 * @brief Starts adaptive k-value.
 */
void DYNUS::startAdaptKValue()
{

  // Compute the average computation time
  for (int i = 0; i < store_computation_times_.size(); i++)
  {
    est_comp_time_ += store_computation_times_[i];
  }
  est_comp_time_ = est_comp_time_ / store_computation_times_.size();

  // Start k_value adaptation
  use_adapt_k_value_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the subgoal.
 * @param const state &A: starting state.
 * @param const state &G_term: goal state.
 * @return bool
 */
void DYNUS::computeG(const state &A, const state &G_term, double horizon)
{

  // Initialize the result
  state local_G;

  // Compute pos for G
  local_G.pos = dynus_utils::projectPointToSphere(A.pos, G_term.pos, horizon);

  // Compute yaw for G
  Eigen::Vector3d dir = (G_term.pos - local_G.pos).normalized();
  local_G.yaw = atan2(dir[1], dir[0]);

  // Set G
  setG(local_G);

  // If we further want to find a G point that is in free space

  // mtx_plan_.lock();
  // state last_plan_state = plan_.back();
  // mtx_plan_.unlock();

  // // Find the closest free point to the projected point
  // state G_closest;
  // dgp_manager_.findClosestFreePoint(G.pos, G_closest.pos);
  // mtx_G_.lock();
  // G_ = G_closest;
  // mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Check if the plan_ (future trajectory) is safe
 */
void DYNUS::checkFuturePlanSafety()
{

  // Check if we need to replan
  if (!checkReadyToReplan())
    return;

  // get local plan
  mtx_plan_.lock();
  std::deque<state> local_plan = plan_;
  mtx_plan_.unlock();

  // If the plan's size is less than 2, we don't need to check
  if (local_plan.size() < 2)
    return;

  // get local trajs_
  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  // If there's no traj, we don't need to check
  if (local_trajs.empty())
    return;

  // Get the start and end time
  double start_time = local_plan.front().t;
  double end_time = local_plan.back().t;

  // initialize the result
  bool result = true;

  // Loop through the plan
  Eigen::Vector3d collision_prune_traj_pos;
  double collision_prune_traj_time;
  for (double t = start_time; t < end_time; t += par_.safety_check_dt)
  {

    // Get the index of the state at time t
    int idx = int((t - start_time) / par_.dc);

    // Make sure the index is within the bounds
    if (idx >= local_plan.size())
      idx = local_plan.size() - 1;

    // Get the agent position at time t
    Eigen::Vector3d agent_pos = local_plan[idx].pos;

    // Check if the state is safe
    // loop through the trajs_
    for (const auto &traj : local_trajs)
    {
      // Get the traj state at time t
      Eigen::Vector3d traj_pos = traj->eval(t);

      // Check if the state is safe
      // Check if the distance is less than the safety distance
      if ((abs(agent_pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj->bbox[0]) / 2.0) && (abs(agent_pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj->bbox[1]) / 2.0) && (abs(agent_pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj->bbox[2]) / 2.0))
      {
        std::cout << red << bold << "Future collision detected" << reset << std::endl;
        result = false;
        collision_prune_traj_pos = traj_pos;
        collision_prune_traj_time = t;
        break;
      }
    }

    if (!result)
      break;
  }

  // If the plan is safe, we continue with the plan
  if (result)
    return;

  // If the plan is not safe, we need to switch to a contingency plan
  switchToSafePath(collision_prune_traj_pos, collision_prune_traj_time);
}

// ----------------------------------------------------------------------------

/**
 * @brief switch to a safe path that starts from the point closest to the current position
 */
void DYNUS::switchToSafePath(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time)
{

  // Get the current state
  state local_state;
  getState(local_state);

  // Find the closest safe path starting point
  mtx_plan_.lock();
  mtx_plan_safe_paths_.lock();

  bool safe_path_found = false;
  int closest_safe_path_idx = 0;
  for (int i = 0; i < plan_safe_paths_.size(); i++)
  {
    if (!plan_safe_paths_[i].empty())
    {

      // get the first and last point of plan_safe_paths_[i]
      state first_point = plan_safe_paths_[i].front();
      state last_point = plan_safe_paths_[i].back();

      // get local trajs_
      std::vector<std::shared_ptr<dynTraj>> local_trajs;
      getTrajs(local_trajs);

      // check if this safe path is safe
      bool safe_path_is_safe = true;
      for (const auto &traj : local_trajs)
      {
        // check if the first point and last point of the safe path is safe against first and last point of the trajs_
        // Note: We only check the first and last point of the safe path because time is critical here
        for (int j = 0; j < 2; j++)
        {
          Eigen::Vector3d traj_pos;
          if (j == 0)
          {
            traj_pos = traj->pwp.eval(traj->pwp.times.front());
          }
          else
          {
            traj_pos = traj->pwp.eval(traj->pwp.times.back());
          }

          // Check if the distance is less than the safety distance
          if ((abs(first_point.pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj->bbox[0]) / 2.0) && (abs(first_point.pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj->bbox[1]) / 2.0) && (abs(first_point.pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj->bbox[2]) / 2.0) || (abs(last_point.pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj->bbox[0]) / 2.0) && (abs(last_point.pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj->bbox[1]) / 2.0) && (abs(last_point.pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj->bbox[2]) / 2.0))
          {
            safe_path_is_safe = false;
            break;
          }
        }

        if (!safe_path_is_safe)
        {
          break;
        }
      }

      if (safe_path_is_safe)
      {
        safe_path_found = true;
        closest_safe_path_idx = i;
        break;
      }
    }
  }

  if (!safe_path_found)
  {
    std::cout << red << bold << "Future collision detected but no safe path found so generate a contingency plan" << reset << std::endl;
    generateContingencyPlan(collision_prune_traj_pos, collision_prune_traj_time); // Assumes that the mutex mtx_plan_ and mtx_plan_safe_paths_ are locked
    mtx_plan_.unlock();
    mtx_plan_safe_paths_.unlock();
    return;
  }

  std::cout << green << bold << "Future collision detected so swith to safe path" << reset << std::endl;

  // Now we have the closest safe path starting point so replace the plan with the safe path
  plan_.erase(plan_.begin() + closest_safe_path_idx, plan_.end());
  plan_.insert(plan_.end(), plan_safe_paths_[closest_safe_path_idx].begin(), plan_safe_paths_[closest_safe_path_idx].end());

  // Clear the plan_safe_paths_ and resize it as the same size as the plan_
  // plan_safe_paths_.clear();
  plan_safe_paths_.resize(plan_.size(), std::vector<state>());

  mtx_plan_.unlock();
  mtx_plan_safe_paths_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Generates a contingency plan - this should be quicker than par_.dc so we can change the plan immediately
 * @note Assumes that the mutex mtx_plan_ and mtx_plan_safe_paths_ are locked
 */
void DYNUS::generateContingencyPlan(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time)
{

  // Get the next goal from plan_
  state start = plan_.front();

  // Container for potential goals
  std::vector<state> potential_goals;

  // Compute the safe path distance
  double safe_path_distance = par_.min_safe_path_distance +
                              (par_.max_safe_path_distance - par_.min_safe_path_distance) * (start.vel.norm() / par_.v_max);

  // Compute the center goal along the velocity direction
  // Assuming start.pos is of a vector type (e.g. Eigen::Vector3d)
  Eigen::Vector3d velocity_dir = start.vel.normalized();
  Eigen::Vector3d center_goal = start.pos + safe_path_distance * velocity_dir;

  // Define a plane perpendicular to velocity.
  // Use a global 'up' vector (e.g., (0,0,1)). If velocity is almost vertical, we need to choose another reference
  Eigen::Vector3d global_up(0, 0, 1);
  if (std::fabs(velocity_dir.dot(global_up)) > 0.99)
  {
    global_up = Eigen::Vector3d(1, 0, 0);
  }

  // Compute a left vector by taking the cross product of velocity and global up.
  Eigen::Vector3d left_vector = velocity_dir.cross(global_up).normalized();

  // Compute an "up" vector on the plane (perpendicular to both velocity and left)
  Eigen::Vector3d up_vector = velocity_dir.cross(left_vector).normalized();

  // Add the center goal as one potential goal
  state center_state;
  center_state.pos = center_goal;
  potential_goals.push_back(center_state);

  // Generate 8 surrounding potential goals:
  // Directions: 0° (left), 45° (up-left), 90° (up), 135° (up-right),
  // 180° (right), 225° (down-right), 270° (down), 315° (down-left)
  for (int i = 0; i < 8; ++i)
  {
    double angle = (M_PI / 4.0) * i; // angle in radians
    // Compute the offset vector in the plane using the left and up vectors.
    Eigen::Vector3d offset = par_.contingency_lateral_offset * (std::cos(angle) * left_vector + std::sin(angle) * up_vector);

    state goal_state;
    goal_state.pos = center_goal + offset;
    potential_goals.push_back(goal_state);
  }

  // Identify the potential goal that is farthest from the collision prune trajectory and try to plan to that goal
  // If not successful, try the next farthest goal

  // Order the potential goals by distance from the collision prune trajectory
  std::vector<std::pair<double, state>> potential_goals_with_distance;
  for (auto goal : potential_goals)
  {
    double distance = (goal.pos - collision_prune_traj_pos).norm();
    potential_goals_with_distance.push_back(std::make_pair(distance, goal));
  }

  // Sort the potential goals by distance
  std::sort(potential_goals_with_distance.begin(), potential_goals_with_distance.end(),
            [](const std::pair<double, state> &a, const std::pair<double, state> &b)
            { return a.first > b.first; });

  // Try to plan to the farthest goal
  bool plan_success = false;
  std::vector<state> contingency_path;

  for (auto goal : potential_goals_with_distance)
  {

    // clear the contingency path
    contingency_path.clear();

    // Plan to the goal using the closed-form solution
    plan_success = solveClosedForm(start, goal.second, contingency_path, contingency_traj_solver_ptr_, 0.2); // can use any safe_traj_solver_ptr

    // If planning is successful, break
    if (plan_success)
      break;
  }

  // Update the plan_
  if (plan_success)
  {
    // Update the plan_
    plan_.clear();
    plan_.insert(plan_.end(), contingency_path.begin(), contingency_path.end());
  }
  else
  {
    // If planning is not successful, we will just stop the drone
    state stop_state;
    stop_state.pos = start.pos;
    stop_state.vel = Eigen::Vector3d::Zero();
    stop_state.accel = Eigen::Vector3d::Zero();
    stop_state.jerk = Eigen::Vector3d::Zero();
    stop_state.yaw = start.yaw;
    stop_state.dyaw = 0.0;
    stop_state.t = start.t;
    plan_.clear();
    plan_.push_back(stop_state);
  }

  // Clear the plan_safe_paths_ and resize it as the same size as the plan_
  // plan_safe_paths_.clear();
  // plan_safe_paths_.resize(plan_.size(), std::vector<state>());
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if we need to replan.
 * @return bool
 */

bool DYNUS::needReplan(const state &local_state, const state &local_G_term, const state &last_plan_state)
{

  // Compute the distance to the terminal goal
  double dist_to_term_G = (local_state.pos - local_G_term.pos).norm();
  double dist_from_last_plan_state_to_term_G = (last_plan_state.pos - local_G_term.pos).norm();

  if (par_.dist_to_term_g_verbose)
    std::cout << "dist_to_term_G: " << dist_to_term_G << std::endl;

  if (dist_to_term_G < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    return false;
  }

  if (dist_to_term_G < par_.goal_seen_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN); // This triggers to use the hard final state constraint
    // We observed when the goal is close, the soft final state constraint fails - so we will use MIQP
  }

  if (drone_status_ == DroneStatus::GOAL_SEEN && dist_from_last_plan_state_to_term_G < par_.goal_radius)
  {
    // If you try to replan when the drone is really close to the goal, it will fail. so we will just return false
    return false;
  }

  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING))
    return false;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::findAandAtime(state &A, double &A_time, double current_time, double last_replaning_computation_time)
{

  mtx_plan_.lock();
  int plan_size = plan_.size();
  mtx_plan_.unlock();

  if (plan_size == 0)
  {
    std::cout << bold << red << "plan_size == 0" << reset << std::endl;
    return false;
  }

  if (par_.use_state_update)
  {
    // Change k_value dynamically
    // To get stable results, we will use a default value of k_value until we have enough computation time
    if (!use_adapt_k_value_)
    {
      // Use default k_value
      k_value_ = std::max((int)plan_size - par_.default_k_value, 0);

      // Store computation times
      if (num_replanning_ != 1) // Don't store the very first computation time (because we don't have a previous computation time)
        store_computation_times_.push_back(last_replaning_computation_time);
    }
    else
    {

      // Computation time filtering
      est_comp_time_ = par_.alpha_k_value_filtering * last_replaning_computation_time + (1 - par_.alpha_k_value_filtering) * est_comp_time_;

      // Get state number based on est_comp_time_ and dc
      k_value_ = std::max((int)plan_size - (int)(par_.k_value_factor * est_comp_time_ / par_.dc), 0);
    }

    // Check if k_value_ is valid
    if (plan_size - 1 - k_value_ < 0 || plan_size - 1 - k_value_ >= plan_size)
    {
      k_value_ = plan_size - 1; // If k_value_ is larger than the plan size, we set it to the last state
    }

    // Get A
    mtx_plan_.lock();
    A = plan_[plan_size - 1 - k_value_];
    mtx_plan_.unlock();

    // Get A_time
    A_time = current_time + (plan_size - 1 - k_value_) * par_.dc; // time to A from current_pos is (plan_size - 1 - k_value_) * par_.dc;
  }
  else // If we don't update state - this is for global planner benchmarking purposes
  {
    // Get state
    getState(A);
    A_time = current_time;
  }

  // Check if A is within the map (especially for z)
  if (A.pos[2] < par_.z_min || A.pos[2] > par_.z_max, A.pos[0] < par_.x_min || A.pos[0] > par_.x_max, A.pos[1] < par_.y_min || A.pos[1] > par_.y_max)
  {
    printf("A (%f, %f, %f) is out of the map\n", A.pos[0], A.pos[1], A.pos[2]);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::filterStaticPushPoints(const Vec3f &latest_mean_push_point, const vec_Vecf<3> &global_path, bool found_static_push_points)
{

  // If there's new static push points, we cluster/filter them
  if (found_static_push_points)
  {

    // Double check
    if (!checkIfPointWithinMap(latest_mean_push_point) || !checkIfPointOccupied(latest_mean_push_point))
      return;

    // If the static_push_points_ is empty, we add the latest_mean_push_point and return
    if (static_push_points_.empty())
    {
      static_push_points_.push_back(latest_mean_push_point);
      return;
    }

    // static_push_points_ stores all the push points, so we check if any of them are close, we compute the mean and add it to static_push_points_, and remove the old points. If not, we just add the latest_mean_push_point
    bool is_close = false;
    for (size_t i = 0; i < static_push_points_.size(); i++)
    {
      // Check if the latest_mean_push_point is close to any of the points in static_push_points_
      if ((latest_mean_push_point - static_push_points_[i]).norm() < par_.static_push_clustering_threshold)
      {
        // Compute the mean and check if the mean point is not in free space, if it is, we update the point in static_push_points_
        Vec3f mean_point = (latest_mean_push_point + static_push_points_[i]) / 2.0;

        if (checkIfPointOccupied(mean_point) && checkIfPointWithinMap(mean_point))
        {
          static_push_points_[i] = mean_point;
          is_close = true;
          break;
        }
      }
    }

    // If the latest_mean_push_point is not close to any of the points in static_push_points_, we add it
    if (!is_close)
      static_push_points_.push_back(latest_mean_push_point);
  }

  // Also we go through the global path and check if any of the path points are close to points stored in static_push_points_, and if not, we remove them. This is important because we don't want to keep old points that are not close to the path
  vec_Vecf<3> new_static_push_points;

  // Loop through the static_push_points_
  for (size_t i = 0; i < static_push_points_.size(); i++)
  {
    bool is_close_to_path = false;
    for (size_t j = 0; j < global_path.size(); j++)
    {
      if ((static_push_points_[i] - global_path[j]).norm() < par_.max_dist_threshold_for_static_push)
      {
        is_close_to_path = true;
        break;
      }
    }

    // If the point is close to the path, we keep it
    if (is_close_to_path)
    {
      new_static_push_points.push_back(static_push_points_[i]);
    }
  }

  // Loop through the static push_points_ and remove the points that are not in occupied space
  static_push_points_.clear();
  for (size_t i = 0; i < new_static_push_points.size(); i++)
  {
    if (checkIfPointOccupied(new_static_push_points[i]) && checkIfPointWithinMap(new_static_push_points[i]))
    {
      static_push_points_.push_back(new_static_push_points[i]);
    }
  }
}

void DYNUS::staticPush(vec_Vecf<3> &global_path)
{
  // Get the static push vectors for each segment; if none found, return.
  Vecf<3> mean_push_point = Vecf<3>::Zero();
  bool found_static_push_points = dgp_manager_.computeStaticPushPoints(global_path, par_.dist_discretization, mean_push_point, par_.num_lookahead_global_path_for_push);

  // Filter the static push points.
  filterStaticPushPoints(mean_push_point, global_path, found_static_push_points);

  // If there are no static push points, return.
  if (static_push_points_.empty())
    return;

  // Process each global path point (skip the first point, which remains unchanged).
  for (size_t i = 1; i < global_path.size(); i++)
  {
    // Compute the cumulative push vector from all static push points.
    Eigen::Vector3d total_push_vector = Eigen::Vector3d::Zero();
    int valid_push_count = 0;

    for (size_t j = 0; j < static_push_points_.size(); j++)
    {
      Eigen::Vector3d push_vector = global_path[i] - static_push_points_[j];
      double dist = push_vector.norm();
      // Only include static push points that are within the allowed threshold.
      if (dist <= par_.max_dist_threshold_for_static_push)
      {
        total_push_vector += push_vector.normalized();
        valid_push_count++;
      }
    }

    // If no static push point contributed, skip this global path point.
    if (valid_push_count == 0)
      continue;

    // Normalize the cumulative push vector.
    total_push_vector.normalize();

    // Start with the maximum push force and decrease it individually if needed.
    double current_push = par_.push_force_static;
    double step = current_push / 10.0;
    Eigen::Vector3d candidate_point = global_path[i] + current_push * total_push_vector;

    // Decrease the push force until the candidate point is collision-free.
    while (current_push >= 0 &&
           (!checkIfPointFree(candidate_point) || !checkIfPointWithinMap(candidate_point)))
    {
      current_push -= step;
      candidate_point = global_path[i] + current_push * total_push_vector;
    }

    // Sometiems the free point a second ago becomes non-free in the next iteration, so we will decrease the push force one more step if it's not already 0
    if (current_push > 0)
    {
      current_push -= step;
      candidate_point = global_path[i] + current_push * total_push_vector;
    }

    candidate_point(2) = global_path[i](2);

    // Update the global path point with the safe candidate.
    global_path[i] = candidate_point;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointOccupied(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointWithinMap(const Vec3f &point)
{
  // Check if the point is free
  return (point.x() >= par_.x_min && point.x() <= par_.x_max && point.y() >= par_.y_min && point.y() <= par_.y_max && point.z() >= par_.z_min && point.z() <= par_.z_max);
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointFree(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointFree(point);
}

void DYNUS::dynamicPush(vec_Vecf<3> &global_path, double current_time, std::vector<std::shared_ptr<dynTraj>> &local_trajs)
{

  // Initialize the adjusted path
  vec_Vec3f adjusted_path;

  // Keep the first point unchanged (initial condition)
  adjusted_path.push_back(global_path[0]);

  // Loop through the remaining points in the path
  for (int i = 1; i < global_path.size(); i++)
  {

    // Initialize the adjusted point
    Eigen::Vector3d point = global_path[i];
    Eigen::Vector3d total_repulsion = Eigen::Vector3d::Zero();

    // Accumulate repulsion from each obstacle
    for (const auto &traj : local_trajs)
    {
      // Compute push force for this obstacle
      double push_force = par_.dyn_obst_global_planner_push_k +
                          par_.dyn_obst_global_planner_push_cov_p_alpha * traj->ekf_cov_p.norm();

      // Evaluate obstacle's position at current time
      double traj_x = traj->eval(current_time).x();
      double traj_y = traj->eval(current_time).y();
      double traj_z = traj->eval(current_time).z();
      Eigen::Vector3d obstacle_position(traj_x, traj_y, traj_z);

      // Compute vector from obstacle to point
      Eigen::Vector3d vec_to_point = point - obstacle_position;
      double distance = vec_to_point.norm();

      if (distance < par_.planner_Cw)
      {
        // Avoid division by zero
        if (distance < 1e-6)
          distance = 1e-6;

        Eigen::Vector3d direction = vec_to_point.normalized();
        Eigen::Vector3d repulsion;

        if (distance < 3.0)
        {
          repulsion = par_.dyn_obst_replusion_max * direction;
        }
        else
        {
          repulsion = push_force / distance * direction;
        }

        // Accumulate the repulsion
        total_repulsion += repulsion;
      }
    } // end for each obstacle

    // Ensure the repulsion for each axis is within the maximum limit
    total_repulsion = total_repulsion.cwiseMax(-par_.dyn_obst_replusion_max)
                          .cwiseMin(par_.dyn_obst_replusion_max);

    // total_repulsion(2) = 0.0;

    // Apply the computed total repulsion to the point.
    Eigen::Vector3d pushed_point = point + total_repulsion;
    Eigen::Vector3d original_total_repulsion = total_repulsion;

    // Check for collisions and adjust the repulsion if needed.
    Eigen::Vector3d original_pushed_point = pushed_point;
    for (int k = 1; k <= 10; k++)
    {
      // if (checkIfPointFree(pushed_point) && checkIfPointWithinMap(pushed_point))
      if (!checkIfPointOccupied(pushed_point) && checkIfPointWithinMap(pushed_point))
        break; // Exit loop if collision-free

      // Gradually reduce the overall repulsion and update the pushed point.
      total_repulsion = original_total_repulsion * (1.0 - k / 10.0);
      pushed_point = point + total_repulsion;
    }

    adjusted_path.push_back(pushed_point);
  }

  // Update the global path with the adjusted path.
  global_path = adjusted_path;
}

// ----------------------------------------------------------------------------

bool DYNUS::getSafeCorridor(const vec_Vecf<3> &global_path, const state &A)
{

  // Timer for computing the safe corridor
  MyTimer cvx_decomp_timer(true);

  // Debug
  if (par_.debug_verbose)
    std::cout << "Convex decomposition" << std::endl;

  // Check if the convex decomposition failed
  if (!dgp_manager_.cvxEllipsoidDecomp(A, global_path, safe_corridor_polytopes_whole_, poly_out_whole_))
  {
    std::cout << bold << red << "Convex decomposition failed" << reset << std::endl;
    poly_out_whole_.clear();
    poly_out_safe_.clear();
    return false;
  }

  // Get computation time [ms]
  cvx_decomp_time_ = cvx_decomp_timer.getElapsedMicros() / 1000.0;

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::computeMapSize(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
{

  // Get local_A
  state local_A;
  getA(local_A);

  // Increase the effective buffer size based on the number of DGP failures.
  double dynamic_buffer = par_.map_buffer + par_.failure_map_buffer_increment * dgp_failure_count_;

  // Increase the effective buffer size based on velocity.
  double dynamic_buffer_x = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(local_A.vel[0]);
  double dynamic_buffer_y = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(local_A.vel[1]);
  double dynamic_buffer_z = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(local_A.vel[2]);

  // Compute the distance to the terminal goal for each axis.
  double dist_x = std::abs(min_pos[0] - max_pos[0]);
  double dist_y = std::abs(min_pos[1] - max_pos[1]);
  double dist_z = std::abs(min_pos[2] - max_pos[2]);

  // Update the map size based on the min and max positions.
  wdx_ = std::max(dist_x + 2 * dynamic_buffer_x, par_.min_wdx);
  wdy_ = std::max(dist_y + 2 * dynamic_buffer_y, par_.min_wdy);
  wdz_ = std::max(dist_z + 2 * dynamic_buffer_z, par_.min_wdz);

  // Compute the base map center as the midpoint between the min and max positions.
  map_center_ = (min_pos + max_pos) / 2.0;

  // // Compute the raw shift based on the agent's velocity.
  // Eigen::Vector3d velocity_shift = par_.center_shift_factor * A.vel;

  // // Clip the velocity shift to the maximum allowable shift.
  // Eigen::Vector3d clipped_shift = velocity_shift.cwiseMin(Eigen::Vector3d(par_.map_buffer, par_.map_buffer, par_.map_buffer)).cwiseMax(Eigen::Vector3d(-par_.map_buffer, -par_.map_buffer, -par_.map_buffer));

  // Compute the new map center by applying the clipped shift to the base center.
  // map_center_ = base_center + clipped_shift;
}

// ----------------------------------------------------------------------------

void DYNUS::getCurrentWds(double &wdx, double &wdy, double &wdz)
{
  wdx = wdx_;
  wdy = wdy_;
  wdz = wdz_;
}

// ----------------------------------------------------------------------------

bool DYNUS::checkPointWithinMap(const Eigen::Vector3d &point) const
{
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) && (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) && (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void DYNUS::updateMapRes()
{

  // Check if we need to adapt the map resolution
  if (replanning_failure_count_ < par_.map_res_adaptation_threshold)
  {
    // map_res_ = par_.res;
    return;
  }

  // Decrease the map resolution
  double factor = static_cast<double>(replanning_failure_count_ - par_.map_res_adaptation_threshold) / static_cast<double>(par_.map_res_adaptation_threshold);
  printf("factor: %f\n", factor);
  map_res_ = std::min(map_res_, std::max(par_.res - factor * par_.map_res_adaptation_decrement, par_.dynus_map_res_min));
  printf("map_res_: %f\n", map_res_);

  // Update map resolution in the DGP manager
  dgp_manager_.updateMapRes(map_res_);
}

// ----------------------------------------------------------------------------

void DYNUS::getStaticPushPoints(vec_Vecf<3> &static_push_points)
{
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void DYNUS::getPpoints(vec_Vecf<3> &p_points)
{
  p_points = p_points_;
}

// ----------------------------------------------------------------------------

void DYNUS::getLocalGlobalPath(vec_Vecf<3> &local_global_path, vec_Vecf<3> &local_global_path_after_push)
{
  local_global_path = local_global_path_;
  local_global_path_after_push = local_global_path_after_push_;
}

// ----------------------------------------------------------------------------

void DYNUS::getGlobalPath(vec_Vecf<3> &global_path)
{
  mtx_global_path_.lock();
  global_path = global_path_;
  mtx_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getOriginalGlobalPath(vec_Vecf<3> &original_global_path)
{
  mtx_original_global_path_.lock();
  original_global_path = original_global_path_;
  mtx_original_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getFreeGlobalPath(vec_Vecf<3> &free_global_path)
{
  free_global_path = free_global_path_;
}

// ----------------------------------------------------------------------------

void DYNUS::resetData()
{

  final_g_ = 0.0;
  global_planning_time_ = 0.0;
  dgp_static_jps_time_ = 0.0;
  dgp_check_path_time_ = 0.0;
  dgp_dynamic_astar_time_ = 0.0;
  dgp_recover_path_time_ = 0.0;
  cvx_decomp_time_ = 0.0;
  initial_guess_computation_time_ = 0.0;
  local_traj_computation_time_ = 0.0;
  safe_paths_time_ = 0.0;
  safety_check_time_ = 0.0;
  yaw_sequence_time_ = 0.0;
  yaw_fitting_time_ = 0.0;

  poly_out_whole_.clear();
  poly_out_safe_.clear();
  goal_setpoints_.clear();
  // pwp_to_share_.clear();
  optimal_yaw_sequence_.clear();
  yaw_control_points_.clear();
  yaw_knots_.clear();
  cps_.clear();
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveData(double &final_g,
                         double &global_planning_time,
                         double &dgp_static_jps_time,
                         double &dgp_check_path_time,
                         double &dgp_dynamic_astar_time,
                         double &dgp_recover_path_time,
                         double &cvx_decomp_time,
                         double &initial_guess_computation_time,
                         double &local_traj_computatoin_time,
                         double &safety_check_time,
                         double &safe_paths_time,
                         double &yaw_sequence_time,
                         double &yaw_fitting_time)
{
  final_g = final_g_;
  global_planning_time = global_planning_time_;
  dgp_static_jps_time = dgp_static_jps_time_;
  dgp_check_path_time = dgp_check_path_time_;
  dgp_dynamic_astar_time = dgp_dynamic_astar_time_;
  dgp_recover_path_time = dgp_recover_path_time_;
  cvx_decomp_time = cvx_decomp_time_;
  initial_guess_computation_time = initial_guess_computation_time_;
  local_traj_computatoin_time = local_traj_computation_time_;
  safe_paths_time = safe_paths_time_;
  safety_check_time = safety_check_time_;
  yaw_sequence_time = yaw_sequence_time_;
  yaw_fitting_time = yaw_fitting_time_;
}

// ----------------------------------------------------------------------------

void DYNUS::setObjectTrackingPosition(const Eigen::Vector3d &object_pos)
{

  // Set the object_pos
  object_pos_ = object_pos;

  if (!object_received_)
  {
    // If this is the first time we receive the object position, we set the flag to true
    object_received_ = true;
    std::cout << "Object position received: " << object_pos_.transpose() << std::endl;
  }
  else
  {
    // If we already received the object position, we just update it
    std::cout << "Object position updated: " << object_pos_.transpose() << std::endl;
  }
}

// ----------------------------------------------------------------------------

void DYNUS::retrievePolytopes(vec_E<Polyhedron<3>> &poly_out_whole, vec_E<Polyhedron<3>> &poly_out_safe)
{
  poly_out_whole = poly_out_whole_;
  poly_out_safe = poly_out_safe_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveGoalSetpoints(std::vector<state> &goal_setpoints)
{
  goal_setpoints = goal_setpoints_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveListSubOptGoalSetpoints(std::vector<std::vector<state>> &list_subopt_goal_setpoints)
{
  list_subopt_goal_setpoints = list_subopt_goal_setpoints_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveYawData(std::vector<double> &optimal_yaw_sequence,
                            std::vector<double> &yaw_control_points,
                            std::vector<double> &yaw_knots)
{
  optimal_yaw_sequence = optimal_yaw_sequence_;
  yaw_control_points = yaw_control_points_;
  yaw_knots = yaw_knots_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveCPs(std::vector<Eigen::Matrix<double, 3, 6>> &cps)
{
  cps = cps_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Replans the trajectory.
 * @param double last_replaning_computation_time: Last replanning computation time.
 * @param double current_time: Current timestamp.
 */
std::tuple<bool, bool> DYNUS::replan(double last_replaning_computation_time, double current_time)
{

  /* -------------------- Housekeeping -------------------- */

  // if (par_.debug_verbose)
  MyTimer timer_housekeeping(true);

  // Reset Data
  resetData();

  // Check if we need to replan
  if (!checkReadyToReplan())
  {
    std::cout << bold << red << "Planner is not ready to replan" << reset << std::endl;
    return std::make_tuple(false, false);
  }

  // Map resolution adaptation
  if (par_.use_map_res_adaptation)
    updateMapRes();

  // Get states we need
  state local_state, local_G_term, last_plan_state;
  getState(local_state);
  getGterm(local_G_term);
  getLastPlanState(last_plan_state);

  // Check if we need to replan based on the distance to the terminal goal
  if (!needReplan(local_state, local_G_term, last_plan_state))
    return std::make_tuple(false, false);

  if (par_.debug_verbose)
    std::cout << "Housekeeping: " << timer_housekeeping.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Global Planning -------------------- */

  MyTimer timer_global(true);
  vec_Vecf<3> global_path;
  if (!generateGlobalPath(global_path, current_time, last_replaning_computation_time))
  {
    if (par_.debug_verbose)
      std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, false);
  }
  if (par_.debug_verbose)
    std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Local Trajectory Optimization -------------------- */

  MyTimer timer_local(true);
  if (!planLocalTrajectory(global_path))
  {
    if (par_.debug_verbose)
      std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Safety Check -------------------- */

  // MyTimer timer_safety(true);
  // if (!safetyCheck(pwp_to_share_, current_time))
  // { // current_time is used to choose the right obstacle trajectory
  //   std::cout << bold << red << "Safety check failed" << reset << std::endl;
  //   replanning_failure_count_++;
  //   if (par_.debug_verbose)
  //     std::cout << "Safety Check: " << timer_safety.getElapsedMicros() / 1000.0 << " ms" << std::endl;
  //   return std::make_tuple(false, true);
  // }
  // if (par_.debug_verbose)
  //   std::cout << "Safety Check: " << timer_safety.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Safe Paths -------------------- */

  // MyTimer timer_safe(true);
  // std::vector<std::vector<state>> safe_path_points;
  // if (!planSafePaths(safe_path_points))
  // {
  //   if (par_.debug_verbose)
  //     std::cout << "Safe Paths: " << timer_safe.getElapsedMicros() / 1000.0 << " ms" << std::endl;
  //   return std::make_tuple(false, true);
  // }
  // if (par_.debug_verbose)
  //   std::cout << "Safe Paths: " << timer_safe.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Append to Plan -------------------- */

  MyTimer timer_append(true);
  if (!appendToPlan())
  {
    if (par_.debug_verbose)
      std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Final Housekeeping -------------------- */

  MyTimer timer_final(true);
  // For terminal goal mode with frontiers: if the terminal goal is close, then stop using frontiers.
  if (par_.flight_mode == "terminal_goal" && par_.use_frontiers && dgp_manager_.checkIfPointFree(local_G_term.pos))
    use_frontiers_as_G_ = false;

  if (par_.debug_verbose)
    std::cout << bold << green << "Replanning succeeded" << reset << std::endl;

  // Reset the replanning failure count
  replanning_failure_count_ = 0;
  if (par_.debug_verbose)
    std::cout << "Final Housekeeping: " << timer_final.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  return std::make_tuple(true, true);
}

// ----------------------------------------------------------------------------

bool DYNUS::generateGlobalPath(vec_Vecf<3> &global_path, double current_time, double last_replaning_computation_time)
{

  // Get G and G_term
  state local_G, local_G_term;
  getG(local_G);
  getGterm(local_G_term);

  // Declare local variables
  state local_A;
  double A_time;

  // Find A and A_time
  if (!findAandAtime(local_A, A_time, current_time, last_replaning_computation_time))
  {
    replanning_failure_count_++;
    return false;
  }

  // Set A and A_time
  setA(local_A);
  setA_time(A_time);

  // Compute G
  if (par_.flight_mode == "terminal_goal")
  {
    computeG(local_A, local_G_term, par_.horizon);
  }
  else if (par_.flight_mode == "exploration" || use_frontiers_as_G_)
  {
    state frontier_G;
    getBestFrontier(frontier_G);

    mtx_G_.lock();
    G_.pos = dynus_utils::projectPointToSphere(local_A.pos, frontier_G.pos, par_.horizon);
    mtx_G_.unlock();
  }

  // Set up the DGP planner (since updateVmax() needs to be called after setupDGPPlanner, we use v_max_ from the last replan)
  dgp_manager_.setupDGPPlanner(par_.global_planner, par_.global_planner_verbose, map_res_, v_max_, par_.a_max, par_.j_max, par_.dgp_timeout_duration_ms);

  // Free start and goal if necessary
  if (par_.use_free_start)
    dgp_manager_.freeStart(local_A.pos, par_.free_start_factor);
  if (par_.use_free_goal)
    dgp_manager_.freeGoal(local_G.pos, par_.free_goal_factor);

  // Debug
  if (par_.debug_verbose)
    std::cout << "Solving DGP" << std::endl;

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
    local_G.pos[2] = 1.0;
  }

  // 1) Build a direction hint from the *previous* global path
  vec_Vecf<3> prev_global;
  getGlobalPath(prev_global); // last successful global path

  Eigen::Vector3d dir_hint = Eigen::Vector3d::Zero();
  if (prev_global.size() >= 2)
  {
    Eigen::Vector3d s0 = prev_global[0];
    Eigen::Vector3d s1 = prev_global[1];
    Eigen::Vector3d seg = s1 - s0;
    if (seg.norm() > 1e-8)
    {
      dir_hint = seg.normalized();
    }
  }

  // Keep ground robots planar
  if (par_.vehicle_type != "uav")
    dir_hint[2] = 0.0;

  // 2) Use this as the "start_vel" argument (magnitude doesn't matter; we use the direction)
  Vec3f start_dir_hint(dir_hint.x(), dir_hint.y(), dir_hint.z());

  // Solve DGP
  // if (!dgp_manager_.solveDGP(local_A.pos, local_A.vel, local_G.pos, final_g_, par_.global_planner_huristic_weight, A_time, global_path))
  if (!dgp_manager_.solveDGP(local_A.pos, start_dir_hint, local_G.pos,
    final_g_, par_.global_planner_huristic_weight,
    A_time, global_path))
  {
    if (par_.debug_verbose)
      std::cout << bold << red << "DGP did not find a solution" << reset << std::endl;
    dgp_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // use this for map resizing
  mtx_global_path_.lock();
  global_path_ = global_path;
  mtx_global_path_.unlock();

  // For visualization
  mtx_original_global_path_.lock();
  original_global_path_ = global_path;
  mtx_original_global_path_.unlock();

  // Debug
  if (par_.debug_verbose)
    std::cout << "global_path.size(): " << global_path.size() << std::endl;

  // Get computation time
  dgp_manager_.getComputationTime(global_planning_time_, dgp_static_jps_time_, dgp_check_path_time_, dgp_dynamic_astar_time_, dgp_recover_path_time_);

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::planLocalTrajectory(vec_Vecf<3> &global_path)
{

  // Get local_A, local_G and A_time
  state local_A, local_G;
  double A_time;
  getA(local_A);
  getG(local_G);
  getA_time(A_time);

  // We will only need par_.num_N polytopes from the global path
  if (global_path.size() > par_.num_N + 1) // + 1 because path has one more points than the number of polytopes
  {
    global_path.erase(global_path.begin() + par_.num_N + 1, global_path.end());
  }

  // Detect the point that intersects with unknown map (if any) and generate a new global path if necessary
  // trimPathAtFirstMapCollision(global_path);

  // If the global path's size is < 3 after trimming, we cannot proceed
  if (global_path.empty() || global_path.size() < 3)
  {
    std::cout << bold << red << "Global path's size is < 3 after trimming" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // convex decomposition
  if (!getSafeCorridor(global_path, local_A))
  {
    replanning_failure_count_++;
    return false;
  }

  // Debug
  if (par_.debug_verbose)
    std::cout << "Gurobi Solver setup" << std::endl;

  /*
   * Local Trajectory Optimization
   */

  if (par_.debug_verbose)
    std::cout << "Safe trajectory solver setup" << std::endl;

  // Initialize flag
  bool optimization_succeeded = false;

  if (par_.debug_verbose)
    std::cout << "generateLocalTrajectory" << std::endl;

  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
  }

  optimization_succeeded = generateLocalTrajectory(
      local_A, A_time, global_path, initial_guess_computation_time_,
      local_traj_computation_time_, whole_traj_solver_ptr_);

  if (par_.debug_verbose)
  {
    std::cout << "initial_guess_computation_time_: " << initial_guess_computation_time_ << " ms" << std::endl;
    std::cout << "Local trajectory optimization finished" << std::endl;
  }

  if (optimization_succeeded)
  {

    // For the optimal solution
    whole_traj_solver_ptr_->reconstructPVATCPopt(zopt_); // First recover the final control points and times from z_opt
    whole_traj_solver_ptr_->getGoalSetpoints(goal_setpoints_);

    // print out the goal setpoints
    // whole_traj_solver_ptr_->getPieceWisePol(pwp_to_share_);
    whole_traj_solver_ptr_->getControlPoints(cps_); // Bezier control points

    // Get goal setpoints for suboptimal solutions for visualization
    if (par_.use_multiple_initial_guesses)
    {

      if (par_.debug_verbose)
        std::cout << "Size of list_z_subopt_: " << list_z_subopt_.size() << std::endl;

      // Initialize list_subopt_goal_setpoints_
      list_subopt_goal_setpoints_.clear();

      // Loop over list_z_subopt
      for (int idx = 0; idx < list_z_subopt_.size(); ++idx)
      {
        // Reconstruct control points and times for each suboptimal solution
        whole_traj_solver_ptr_->reconstructPVATCPopt(list_z_subopt_[idx]); // First recover the final control points and times from z_opt
        std::vector<state> subopt_goal_setpoints;
        whole_traj_solver_ptr_->getGoalSetpoints(subopt_goal_setpoints);
        list_subopt_goal_setpoints_.push_back(subopt_goal_setpoints);
      }
    }
  }
  else
  {
    replanning_failure_count_++;
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::trimPathAtFirstMapCollision(vec_Vecf<3> &input_path)
{
  bool collision = false;
  vec_Vecf<3> trimmed_path;

  // Temporary storage for one-neighbor search
  std::vector<int> indices(1);
  std::vector<float> dists2(1);
  int found = 0;

  for (size_t i = 0; i < input_path.size(); ++i)
  {
    // Convert Eigen point to PCL point
    pcl::PointXYZ query;
    query.x = input_path[i].x();
    query.y = input_path[i].y();
    query.z = input_path[i].z();

    {
      // Nearest neighbor search
      std::lock_guard<std::mutex> lk(mtx_kdtree_unk_); // Ensure thread safety if needed
      found = kdtree_unk_.nearestKSearch(query, 1, indices, dists2);
    }
    if (found > 0)
    {
      float dist = std::sqrt(dists2[0]);
      if (dist < par_.drone_radius)
      {
        // Collision!
        collision = true;
        if (i == 0)
        {
          // First point collides: keep it so path isn't empty
          trimmed_path.push_back(input_path[0]);
        }
        break;
      }
    }

    // No collision at this point: include it
    trimmed_path.push_back(input_path[i]);
  }

  if (collision)
  {
    // If a collision was detected, we trim the path at the first collision point
    input_path = trimmed_path;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::generateLocalTrajectory(const state &local_A, double A_time,
                                    vec_Vec3f &global_path,
                                    double &initial_guess_computation_time,
                                    double &local_traj_computation_time,
                                    std::shared_ptr<lbfgs::SolverLBFGS> &whole_traj_solver_ptr)
{

  if (par_.debug_verbose)
    std::cout << "Preparing solver for replan" << std::endl;

  // Initialize the solver.

  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  // Get the last safe corridor polytope's mean point and make it Local_E
  state local_E;
  local_E.pos = global_path.back();

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_E.pos[2] = 1.0;
  }

  whole_traj_solver_ptr->prepareSolverForReplan(A_time, global_path, safe_corridor_polytopes_whole_, local_trajs, local_A, local_E, initial_guess_computation_time, par_.use_multiple_initial_guesses);

  // It's pushed in prepareSolverForReplan() so we get the pushed global path
  whole_traj_solver_ptr->getGlobalPath(global_path);

  mtx_global_path_.lock();
  global_path_ = global_path; // Update the global path
  mtx_global_path_.unlock();

  // update local_E
  local_E.pos = global_path.back();
  mtx_E_.lock();
  E_ = local_E; // Update the local_E
  mtx_E_.unlock();

  if (par_.debug_verbose)
    std::cout << "Solver prepared" << std::endl;

  // Get initial guesses
  auto list_z0 = whole_traj_solver_ptr_->getInitialGuesses();
  auto list_initial_guess_wps = whole_traj_solver_ptr_->getInitialGuessWaypoints();

  if (par_.debug_verbose)
    std::cout << "Initial guesses size: " << list_z0.size() << std::endl;

  // Update L-BFGS parameters.
  lbfgs_params_.mem_size = static_cast<int>(list_z0[0].size());

  // Prepare vectors for parallelization
  int status = -1; // Initialize status
  int size_of_list_z0 = list_z0.size();

  std::vector<int> list_status(size_of_list_z0, -1); // Initialize status for each thread
  std::vector<Eigen::VectorXd> list_zopt(size_of_list_z0);
  std::vector<double> list_fopt(size_of_list_z0, 0.0);
  std::vector<double> list_initial_guess_computation_time(size_of_list_z0);

  // Solve the optimiation problem.

  if (!par_.use_multiple_initial_guesses || size_of_list_z0 == 1)
  {
    auto t_start = std::chrono::high_resolution_clock::now();
    status = whole_traj_solver_ptr_->optimize(list_z0[0], zopt_, fopt_, lbfgs_params_);
    auto t_end = std::chrono::high_resolution_clock::now();
    local_traj_computation_time = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    // std::cout << lbfgs::lbfgs_strerror(status) << std::endl;

    // status = 10;
    // zopt_ = list_z0[0]; // Initialize zopt_ with the correct size
    // fopt_ = 0.0; // Initialize fopt_ to zero
    list_initial_guess_wps_subopt_.clear();
    list_initial_guess_wps_subopt_.push_back(list_initial_guess_wps[0]); // Store the initial guess waypoints for the optimal solution
  }
  else
  {
    // Parallelization approach (is there any faster way to do this?)
    std::vector<std::future<void>> futures;
    futures.reserve(size_of_list_z0);

    auto t_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < size_of_list_z0; ++i)
    {
      futures.emplace_back(std::async(std::launch::async,
                                      [&, i]()
                                      {
                                        // make a fresh solver for thread-safety
                                        std::shared_ptr<lbfgs::SolverLBFGS> solver_ptr = std::make_shared<lbfgs::SolverLBFGS>();
                                        solver_ptr->initializeSolver(planner_params_);
                                        double initial_guess_computation_time = 0.0;
                                        solver_ptr->prepareSolverForReplan(A_time, global_path, safe_corridor_polytopes_whole_, local_trajs, local_A, local_E, initial_guess_computation_time); // initial time t0 = 0.0

                                        // copy initial guess
                                        Eigen::VectorXd z0 = list_z0[i];
                                        Eigen::VectorXd zopt;
                                        double fopt;

                                        // run optimization
                                        int status = solver_ptr->optimize(z0, zopt, fopt, lbfgs_params_);
                                        list_status[i] = status;
                                        list_zopt[i] = zopt;
                                        list_fopt[i] = fopt;
                                        list_initial_guess_computation_time[i] = initial_guess_computation_time;
                                      }));
    }

    // wait for all to finish
    for (auto &f : futures)
      f.get();

    auto t_end = std::chrono::high_resolution_clock::now();
    local_traj_computation_time = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // Find the best solution
    fopt_ = std::numeric_limits<double>::max();
    zopt_ = list_z0[0]; // Initialize zopt_ with the correct size
    int best_index = -1;
    for (size_t i = 0; i < size_of_list_z0; ++i)
    {
      // First check the status
      if (list_status[i] < 0)
      {
        std::cout << "list_status[" << i << "] = " << list_status[i] << ", skipping this solution" << std::endl;
        std::cout << lbfgs::lbfgs_strerror(list_status[i]) << std::endl;
        continue; // Skip this solution if it failed
      }

      if (list_fopt[i] < fopt_)
      {
        fopt_ = list_fopt[i];
        zopt_ = list_zopt[i];
        best_index = i;
      }
    }

    initial_guess_computation_time = list_initial_guess_computation_time[best_index]; // Get the initial guess computation time for the best solution

    // If no solution was found, return
    if (best_index < 0)
    {
      std::cout << bold << red << "No solution found in multiple initial guesses" << reset << std::endl;
      replanning_failure_count_++;
      initial_guess_computation_time = -100000.0;
      return false;
    }

    list_z_subopt_.clear();                 // Clear the suboptimal solutions list
    list_initial_guess_wps_subopt_.clear(); // Clear the suboptimal waypoints list

    // Grab sub optimal solutions for visualization (solutions that are valid (so no negative status and no crazy big f) but no better than the best one)
    for (size_t i = 0; i < size_of_list_z0; ++i)
    {
      if (list_status[i] >= 0 && i != best_index && list_fopt[i] < planner_params_.BIG * 0.9)
      {
        // Add the solution to the trajs_ for visualization
        list_z_subopt_.push_back(list_zopt[i]);

        // Add the initial guess waypoints for visualization
        list_initial_guess_wps_subopt_.push_back(list_initial_guess_wps[i]);
      }
    }

    // Set the status to the best one found
    status = list_status[best_index];

  } // End of parallelization

  if (par_.debug_verbose)
    std::cout << "Optimization status: " << status << ", fopt: " << fopt_ << ", computation time: " << local_traj_computation_time << " ms" << std::endl;

  // If no solution is found, return.
  if (status < 0 || fopt_ > par_.fopt_threshold)
  {
    // do the same output in red with printf
    printf("\033[1;31mLocal Optimization Failed with status: %d, fopt: %.2f\033[0m\n", status, fopt_);
    return false;
  }

  // If the optimization succeeded, we can update the goal setpoints in green
  // printf("\033[1;32mLocal Optimization Succeeded with status: %d, fopt: %.2f\033[0m\n", status, fopt_);
  return true;
}

bool DYNUS::planSafePaths(std::vector<std::vector<state>> &safe_path_points)
{
  MyTimer safe_paths_timer(true);

  std::map<int, std::vector<state>> safe_paths_in_whole_traj_indexed;
  if (!computeSafePaths(goal_setpoints_, safe_paths_in_whole_traj_indexed))
  {
    std::cout << bold << red << "Not enough safe paths computed" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // Get the safe paths computation time
  safe_paths_time_ = safe_paths_timer.getElapsedMicros() / 1000.0;

  // Print the time for computing safe paths
  if (par_.debug_verbose)
    std::cout << "Safe paths computation time: " << safe_paths_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  // TODO: add safe paths for visualization?

  MyTimer safe_path_points_timer(true);

  // Generate safe path points (which has the same size as goal_setpoints_) has a safe path if the goal setpoint at the same index has the safe path
  if (!generateSafePathPointsandincludeBestSafePath(goal_setpoints_, safe_path_points, safe_paths_in_whole_traj_indexed))
  {
    std::cout << bold << red << "Safe path points generation failed" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // Print the time for computing safe path points
  if (par_.debug_verbose)
    std::cout << "Safe path points generation time: " << safe_path_points_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::appendToPlan()
{

  if (par_.debug_verbose)
    std::cout << "goal_setpoints_.size(): " << goal_setpoints_.size() << std::endl;

  // mutex lock
  mtx_plan_.lock();
  // mtx_plan_safe_paths_.lock();

  // get the size of the plan and plan_safe_paths
  int plan_size = plan_.size();
  // int plan_safe_paths_size = plan_safe_paths_.size();

  // // make sure the size of the plan and plan_safe_paths are the same
  // if (need_to_generate_safe_corridor_for_safe_path_ && plan_size != plan_safe_paths_size)
  // {
  //   std::cout << bold << red << "plan_size != plan_safe_paths_size" << reset << std::endl;
  //   std::cout << "plan_size: " << plan_size << ", plan_safe_paths_size: " << plan_safe_paths_size << std::endl;
  //   mtx_plan_.unlock();
  //   // mtx_plan_safe_paths_.unlock();
  //   replanning_failure_count_++;
  //   return false;
  // }

  // If the plan size is less than k_value_, which means we already passed point A, we cannot use this plan
  if (plan_size < k_value_)
  {
    if (par_.debug_verbose)
      std::cout << bold << red << "(plan_size - k_value_) = " << (plan_size - k_value_) << " < 0" << reset << std::endl;
    k_value_ = std::max(1, plan_size - 1); // Decrease k_value_ to plan_size - 1 but at least 1
  }
  else // If the plan size is greater than k_value_, which means we haven't passed point A yet, we can use this plan
  {
    plan_.erase(plan_.end() - k_value_, plan_.end());
    plan_.insert(plan_.end(), goal_setpoints_.begin(), goal_setpoints_.end());
    // plan_safe_paths_.erase(plan_safe_paths_.end() - k_value_ - 1, plan_safe_paths_.end());
    // plan_safe_paths_.insert(plan_safe_paths_.end(), safe_path_points.begin(), safe_path_points.end());
  }

  // mutex unlock
  // mtx_plan_safe_paths_.unlock();
  mtx_plan_.unlock();

  // k_value adaptation initialization
  if (!got_enough_replanning_)
  {
    if (store_computation_times_.size() < par_.num_replanning_before_adapt)
    {
      num_replanning_++;
    }
    else
    {
      startAdaptKValue();
      got_enough_replanning_ = true;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Compute safe paths using closed-form solution.
 * @param std::vector<state> &goal_setpoints: used to discretize the path.
 * @param std::vector<std::vector<state>> &safe_paths: Output safe paths.
 */
bool DYNUS::computeSafePaths(const std::vector<state> &goal_setpoints, std::map<int, std::vector<state>> &safe_paths)
{

  // Initialize the safe paths
  safe_paths.clear();

  // Compute H point in the goal setpoints
  int h_point_idx = findHPointIndex(goal_setpoints);

  // Get parameters
  int num_increment_goal_setpoints = (int)h_point_idx / par_.num_safe_paths;

  // Sanity check
  if (num_increment_goal_setpoints == 0)
  {
    std::cout << bold << red << "H point is very close" << reset << std::endl;
    return false;
  }

  // Loop through discretized goal setpoints
  // Note disc_idx starts from num_increment_goal_setpoints because we don't need to compute the safe path for the first goal setpoint
  // Also note disc_idx <= h_point_idx because we can compute the safe path for the last goal setpoint
  // for (int disc_idx = num_increment_goal_setpoints; disc_idx <= h_point_idx; disc_idx += num_increment_goal_setpoints)
#pragma omp parallel for
  for (int idx = 0; idx < par_.num_safe_paths; idx++)
  {

    // Get safe_traj_solver_ptr
    std::shared_ptr<ClosedFromSolver> safe_traj_solver_ptr = safe_traj_solver_ptrs_[idx];

    // Get disc_idx
    int disc_idx = idx * num_increment_goal_setpoints;

    // Get the current goal setpoint
    state current_goal_setpoint = goal_setpoints[disc_idx];

    // Compute the safe paths
    std::vector<state> current_safe_path;

    MyTimer safe_path_timer(true);

    // Generate safe paths with closed form
    if (generateSafePathsWithClosedForm(current_goal_setpoint, current_safe_path, safe_traj_solver_ptr))
    {
#pragma omp critical
      {
        safe_paths[disc_idx] = current_safe_path;
      }
    }

    // Print the time for computing the safe path
    if (par_.debug_verbose)
      std::cout << "disc_idx: " << disc_idx << ", Safe path computation time: " << safe_path_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;
  }

  // If there's enough safe paths, return true. But if we planning in only free space, we don't necessarily need to have enough safe paths
  if (!par_.plan_only_in_free_space)
  {
    if (safe_paths.size() >= par_.min_num_safe_paths)
      return true;
    else
      return false;
  }
  else
  {
    return true;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::generateSafePathPointsandincludeBestSafePath(std::vector<state> &goal_setpoints, std::vector<std::vector<state>> &safe_path_points, std::map<int, std::vector<state>> &safe_paths)
{

  // Check if the safe paths is empty
  if (safe_paths.empty())
  {
    return false;
  }

  if (!need_to_generate_safe_corridor_for_safe_path_ || !par_.plan_only_in_free_space) // If planning in unknown space, we don't need to replace the last bit with safe paths
  {
    // Initiliaze safe_path_points with zeros with the same size as goal_setpoints
    safe_path_points.clear();
    safe_path_points.resize(goal_setpoints.size(), std::vector<state>());

    // Retrieve the safe paths
    for (auto it = safe_paths.begin(); it != safe_paths.end(); it++)
    {
      // Append the safe path to safe_path_points
      safe_path_points[it->first] = it->second;
    }

    // Sanity check
    assert(goal_setpoints.size() == safe_path_points.size());

    return true;
  }
  else // If planning in unknown space, we need to replace the last bit with safe paths
  {
    // Initiliaze safe_path_points with zeros with the same size as goal_setpoints
    safe_path_points.clear();
    safe_path_points.resize(goal_setpoints.size(), std::vector<state>());

    // Get the last goal setpoint
    state last_goal_setpoint = goal_setpoints.back();

    // Get the best safe path by looping through the safe paths and find the one that ends closest to the goal.
    double min_distance = std::numeric_limits<double>::max();
    int best_safe_path_idx = -1;
    for (auto it = safe_paths.begin(); it != safe_paths.end(); it++)
    {

      // Append the safe path to safe_path_points
      safe_path_points[it->first] = it->second;

      // Get the last state of the safe path
      state last_safe_state = it->second.back();

      // Compute the distance between the last state of the safe path and the last goal setpoint
      double distance = (last_safe_state.pos - last_goal_setpoint.pos).norm();

      // Update the minimum distance and the best safe path
      if (distance < min_distance)
      {
        min_distance = distance;
        best_safe_path_idx = it->first;
      }
    }

    // if (best_safe_path_idx < 0)
    // {
    //   std::cout << bold << red << "No safe path found" << reset << std::endl;
    //   return false;
    // }

    // Erase anything after best_safe_path_idx
    goal_setpoints.erase(goal_setpoints.begin() + best_safe_path_idx, goal_setpoints.end());
    safe_path_points.erase(safe_path_points.begin() + best_safe_path_idx, safe_path_points.end());

    // Append the best safe path to the goal setpoints
    std::vector<state> best_safe_path = safe_paths[best_safe_path_idx];

    // Append the best safe path to the goal setpoints for the plan
    goal_setpoints.insert(goal_setpoints.end(), best_safe_path.begin(), best_safe_path.end());

    // Add empty vectors to safe_path_points to match the size of goal_setpoints
    for (int i = 0; i < best_safe_path.size(); i++)
    {
      safe_path_points.push_back(std::vector<state>());
    }

    // Sanity check
    assert(goal_setpoints.size() == safe_path_points.size());

    return true;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::generateSafePathsWithClosedForm(const state &start_goal_setpoint, std::vector<state> &safe_path, const std::shared_ptr<ClosedFromSolver> &safe_traj_solver_ptr)
{

  // Clear the safe path
  safe_path.clear();

  // Generate potential goal for start_goal_setpoint
  state potential_goal;
  double closed_form_dt;
  generatePotentialGoal(start_goal_setpoint, potential_goal, closed_form_dt);

  // Check if the potential goal is in safe space
  if (!dgp_manager_.checkIfPointFree(potential_goal.pos))
  {
    if (par_.debug_verbose)
    {
      std::cout << "start_goal_setpoint.pos: " << start_goal_setpoint.pos.transpose() << std::endl;
      std::cout << "potential_goal.pos: " << potential_goal.pos.transpose() << std::endl;
      std::cout << "closed_form_dt: " << closed_form_dt << std::endl;
      std::cout << bold << red << "Potential goal is not in free space" << reset << std::endl;
    }
    return false;
  }

  // If the goal is in safe space, compute the safe path
  // Compute the safe path
  return solveClosedForm(start_goal_setpoint, potential_goal, safe_path, safe_traj_solver_ptr, closed_form_dt);
}

// ----------------------------------------------------------------------------

bool DYNUS::solveClosedForm(const state &start, const state &goal, std::vector<state> &safe_path, const std::shared_ptr<ClosedFromSolver> &safe_traj_solver_ptr, double closed_form_dt)
{

  // If the goal is in safe space, compute the safe path
  // Compute the safe path
  safe_traj_solver_ptr->setX0(start);
  safe_traj_solver_ptr->setXf(goal);
  if (need_to_generate_safe_corridor_for_safe_path_)
    safe_traj_solver_ptr->setPolytopes(safe_corridor_polytopes_safe_);
  else
    safe_traj_solver_ptr->setPolytopes(safe_corridor_polytopes_whole_);
  safe_traj_solver_ptr->setT0(start.t);
  safe_traj_solver_ptr->setClosedFormInitialDt(closed_form_dt);

  // Solve the optimization problem
  if (!safe_traj_solver_ptr->findClosedFormSolution())
    return false;

  // Get the safe path
  safe_traj_solver_ptr->initializeGoalSetpoints();
  safe_traj_solver_ptr->fillGoalSetPoints();
  safe_traj_solver_ptr->getGoalSetpoints(safe_path);

  // TODO: Now we have two sets of trajectory - the first half is the whole trajectory, and the second half is the safe trajectory - I need to make sure that I publish both of them with time stamps
  // TODO: I also need to publish pwp when future collisions are detected and switch to safe trajectory
  // safe_traj_solver_ptr->getPieceWisePol(pwp);

  // Check if the safe path is valid
  if (safe_path.size() > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// ----------------------------------------------------------------------------

void DYNUS::generatePotentialGoal(const state &start_goal_setpoint, state &potential_goal, double &closed_form_dt)
{

  // Initialize the potential goal with zeros
  potential_goal.setZero();

  // Option 1: Using constant jerk model - this could generate very optimistic potential goals
  // // Using constant jerk model, compute minimum time until it stops given the current state
  // double Tx = computeMinimumTimeUntilStop(start_goal_setpoint.vel[0], start_goal_setpoint.accel[0], start_goal_setpoint.jerk[0]);
  // double Ty = computeMinimumTimeUntilStop(start_goal_setpoint.vel[1], start_goal_setpoint.accel[1], start_goal_setpoint.jerk[1]);
  // double Tz = computeMinimumTimeUntilStop(start_goal_setpoint.vel[2], start_goal_setpoint.accel[2], start_goal_setpoint.jerk[2]);

  // // Compute the minimum displacement until it stops (v0 * T + 0.5 * a0 * T^2 - 1/6 * j * T^3)
  // double delta_x = start_goal_setpoint.vel[0] * Tx + 0.5 * start_goal_setpoint.accel[0] * Tx * Tx - 1.0 / 6.0 * par_.j_max * Tx * Tx * Tx;
  // double delta_y = start_goal_setpoint.vel[1] * Ty + 0.5 * start_goal_setpoint.accel[1] * Ty * Ty - 1.0 / 6.0 * par_.j_max * Ty * Ty * Ty;
  // double delta_z = start_goal_setpoint.vel[2] * Tz + 0.5 * start_goal_setpoint.accel[2] * Tz * Tz - 1.0 / 6.0 * par_.j_max * Tz * Tz * Tz;

  // potential_goal.pos[0] = start_goal_setpoint.pos[0] + delta_x;
  // potential_goal.pos[1] = start_goal_setpoint.pos[1] + delta_y;
  // potential_goal.pos[2] = start_goal_setpoint.pos[2] + delta_z;

  // Option 2: given a distance (linearly increased by the velocity), define a stop point in the direction of the current velocity

  // Linearly increase safe_path_distance between min_safe_path_distance and max_safe_path_distance
  double safe_path_distance = par_.min_safe_path_distance + (par_.max_safe_path_distance - par_.min_safe_path_distance) * (start_goal_setpoint.vel.norm() / par_.v_max); // Use par_.v_max instead of v_max_ because this is for safe plan and we should use the maximum possible v_max - should not worry about uncertainty.

  // Estimate the time to reach the potential goal
  closed_form_dt = safe_path_distance / start_goal_setpoint.vel.norm();

  // Update the potential goal
  potential_goal.pos = start_goal_setpoint.pos + safe_path_distance * start_goal_setpoint.vel.normalized();
}

// ----------------------------------------------------------------------------

double DYNUS::computeMinimumTimeUntilStop(double v0, double a0, double j)
{
  // Compute the minimum time until it stops
  // T = (a0 + sqrt(a0^2 + 2 * j * v0)) / j
  return (a0 + std::sqrt(a0 * a0 + 2 * j * v0)) / j;
}

// ----------------------------------------------------------------------------

int DYNUS::findHPointIndex(const std::vector<state> &goal_setpoints)
{

  // Initialize the index
  int h_point_idx = 0;

  // Iterate through the goal setpoints
  for (int i = 0; i < goal_setpoints.size(); i += 10)
  {

    // Get the current goal setpoint
    state current_goal = goal_setpoints[i];

    // Check if the current goal setpoint is safe
    if (dgp_manager_.checkIfPointFree(current_goal.pos))
    {
      h_point_idx = i;
    }
    else
    {
      break;
    }
  }

  return h_point_idx;
}

// ----------------------------------------------------------------------------

/**
 * @brief Check if the newly optimized trajectory is safe
 * @param pwp PieceWisePol
 * @param current_time double
 */
bool DYNUS::safetyCheck(PieceWisePol &pwp, double current_time)
{

  MyTimer safety_check_timer(true);

  // Initialize the flag
  bool result = true;

  // Loop through trajs_
  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  for (auto traj : local_trajs)
  {

    // identify the start time and end time
    double start_time = std::min(pwp.times.front(), traj->pwp.times.front());
    double end_time = std::max(pwp.times.back(), traj->pwp.times.back());

    // Loop through the trajectory
    for (double t = pwp.times.front(); t < pwp.times.back(); t += par_.safety_check_dt)
    {

      // Get the position
      Eigen::Vector3d agent_pos = pwp.eval(t);

      // Get the position of the trajectory
      Eigen::Vector3d traj_pos = traj->eval(t);

      // Check if the distance is less than the safety distance
      if ((abs(agent_pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj->bbox[0]) / 2.0) && (abs(agent_pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj->bbox[1]) / 2.0) && (abs(agent_pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj->bbox[2]) / 2.0))
      {
        result = false;
        break;
      }

    } // end of for loop for t

  } // end of for loop for trajs_

  // Get the safety check time
  safety_check_time_ = safety_check_timer.getElapsedMicros() / 1000.0;

  // Return the result
  return result;
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the static map.
 * @param vec_Vecf<3> &occupied_cells: Output map for occupied cells.
 */
void DYNUS::getMap(vec_Vecf<3> &occupied_cells)
{
  // Get the occupied cells
  dgp_manager_.getOccupiedCells(occupied_cells);
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the static map.
 * @param vec_Vecf<3> &occupied_cells: Output map for occupied cells.
 */
void DYNUS::getFreeMap(vec_Vecf<3> &free_cells)
{
  // Get the free cells
  dgp_manager_.getFreeCells(free_cells);
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the terminal goal state.
 * @param state &G_term: Output terminal goal state.
 */
void DYNUS::getGterm(state &G_term)
{
  mtx_G_term_.lock();
  G_term = G_term_;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the terminal goal state.
 * @param state G_term: Terminal goal state to set.
 */
void DYNUS::setGterm(const state &G_term)
{
  mtx_G_term_.lock();
  G_term_ = G_term;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the subgoal.
 * @param state &G: Output subgoal.
 */
void DYNUS::getG(state &G)
{
  mtx_G_.lock();
  G = G_;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets point E
 * @param state &G: Output point E
 */
void DYNUS::getE(state &E)
{
  mtx_E_.lock();
  E = E_;
  mtx_E_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the subgoal.
 * @param state G: Subgoal to set.
 */
void DYNUS::setG(const state &G)
{
  mtx_G_.lock();
  G_ = G;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets A (starting point for global planning).
 * @param state &G: Output A.
 */
void DYNUS::getA(state &A)
{
  mtx_A_.lock();
  A = A_;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets A (starting point for global planning).
 * @param state &G: Input A.
 */
void DYNUS::setA(const state &A)
{
  mtx_A_.lock();
  A_ = A;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getA_time(double &A_time)
{
  mtx_A_time_.lock();
  A_time = A_time_;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets A (starting point for global planning)'s time
 * @param state &G: Input A time
 */
void DYNUS::setA_time(double A_time)
{
  mtx_A_time_.lock();
  A_time_ = A_time;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the current state.
 * @param state &state: Output current state.
 */
void DYNUS::getState(state &state)
{
  mtx_state_.lock();
  state = state_;
  mtx_state_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the last plan state
 * @param state &state: Output last plan state
 */
void DYNUS::getLastPlanState(state &state)
{
  mtx_plan_.lock();
  state = plan_.back();
  mtx_plan_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets trajs_
 * @param std::vector<std::shared_ptr<dynTraj>> &trajs: Output trajs_
 */
void DYNUS::getTrajs(std::vector<std::shared_ptr<dynTraj>> &out)
{
  std::lock_guard<std::mutex> lock(mtx_trajs_);
  out = trajs_; // copies shared_ptr only, not expressions
}

// ----------------------------------------------------------------------------

/**
 * @brief Cleans up old trajectories.
 * @param double current_time: Current timestamp.
 */
void DYNUS::cleanUpOldTrajs(double current_time)
{
  std::lock_guard<std::mutex> lock(mtx_trajs_);

  // remove_if moves all “expired” to the end, then erase() chops them off
  trajs_.erase(
      std::remove_if(
          trajs_.begin(),
          trajs_.end(),
          [&](const std::shared_ptr<dynTraj> &t)
          {
            return (current_time - t->time_received) > par_.traj_lifetime;
          }),
      trajs_.end());
}

// ----------------------------------------------------------------------------

/**
 * @brief Adds or updates a trajectory.
 * @param dynTraj new_traj: New trajectory to add.
 * @param double current_time: Current timestamp.
 */
void DYNUS::addTraj(std::shared_ptr<dynTraj> new_traj, double current_time)
{

  if (new_traj->mode == dynTraj::Mode::Analytic && !new_traj->analytic_compiled)
  {
    printf("Dropping analytic traj id=%d because compile failed", new_traj->id);
    return;
  }

  // Evaluate once (no copies)
  Eigen::Vector3d p = new_traj->eval(current_time);
  if (!checkPointWithinMap(p))
    return;
  if ((p - state_.pos).norm() > par_.horizon)
    return;

  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    auto it = std::find_if(trajs_.begin(), trajs_.end(),
                           [&](const std::shared_ptr<dynTraj> &t)
                           { return t->id == new_traj->id; });

    if (it != trajs_.end())
      *it = new_traj; // replace pointer
    else
      trajs_.push_back(new_traj);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Updates the current state.
 * @param state data: New state data.
 */
void DYNUS::updateState(state data)
{

  // If we are doing hardware and provide goal in global frame (e.g. vicon), we need to transform the goal to the local frame

  if (par_.use_hardware && par_.provide_goal_in_global_frame)
  {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(data.pos[0], data.pos[1], data.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_ * homo_pos;
    data.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);

    // Apply rotation to velocity
    data.vel = init_pose_transform_rotation_ * data.vel;

    // Apply rotation to accel
    data.accel = init_pose_transform_rotation_ * data.accel;

    // Apply rotation to jerk
    data.jerk = init_pose_transform_rotation_ * data.jerk;

    // Apply yaw
    data.yaw += yaw_init_offset_;
  }

  mtx_state_.lock();
  state_ = data;
  mtx_state_.unlock();

  if (state_initialized_ == false || drone_status_ == DroneStatus::YAWING)
  {

    // create temporary state
    state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    previous_yaw_ = data.yaw;

    // Push the state to the plan
    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    // Push the state to the plan_safe_paths_
    // mtx_plan_safe_paths_.lock();
    // plan_safe_paths_.clear();
    // plan_safe_paths_.push_back(std::vector<state>());
    // mtx_plan_safe_paths_.unlock();

    // Update Point A
    setA(tmp);

    // Update Point G
    setG(tmp);

    // Update the flag
    state_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the next goal (setpoint) from the plan.
 * @param state &next_goal: Output next goal state.
 * @return bool
 */
bool DYNUS::getNextGoal(state &next_goal)
{

  // Check if the planner is initialized
  if (!checkReadyToReplan())
  {
    return false;
  }

  // Pop the front of the plan
  next_goal.setZero();

  // If the plan is empty, return false
  mtx_plan_.lock(); // Lock the mutex
  auto local_plan = plan_;
  mtx_plan_.unlock(); // Unlock the mutex

  // Get the next goal
  next_goal = local_plan.front();

  // If there's more than one goal setpoint, pop the front
  if (local_plan.size() > 1)
  {
    mtx_plan_.lock();
    plan_.pop_front();
    mtx_plan_.unlock();

    // mtx_plan_safe_paths_.lock();
    // plan_safe_paths_.pop_front();
    // mtx_plan_safe_paths_.unlock();
  }

  if (par_.use_hardware && par_.provide_goal_in_global_frame)
  {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(next_goal.pos[0], next_goal.pos[1], next_goal.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_inv_ * homo_pos;

    // Apply transformation to velocity
    Eigen::Vector3d global_vel = init_pose_transform_rotation_inv_ * next_goal.vel;

    // Apply transformation to accel
    Eigen::Vector3d global_accel = init_pose_transform_rotation_inv_ * next_goal.accel;

    // Apply transformation to jerk
    Eigen::Vector3d global_jerk = init_pose_transform_rotation_inv_ * next_goal.jerk;

    next_goal.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);
    next_goal.vel = global_vel;
    next_goal.accel = global_accel;
    next_goal.jerk = global_jerk;
  }

  if (par_.flight_mode == "object_tracking" && object_received_)
  {
    // Look towards the object - compute the optimal yaw
    next_goal.yaw = atan2(object_pos_[1] - next_goal.pos[1], object_pos_[0] - next_goal.pos[0]);
    next_goal.dyaw = (next_goal.yaw - previous_yaw_) / par_.dc;
    next_goal.dyaw = std::clamp(next_goal.dyaw, -par_.w_max, par_.w_max);
    previous_yaw_ = next_goal.yaw;
  }
  else if (!(drone_status_ == DroneStatus::GOAL_REACHED))
  {
    // Get the desired yaw
    // If the planner keeps failing, just keep spinning
    if (replanning_failure_count_ > par_.yaw_spinning_threshold)
    {
      next_goal.yaw = previous_yaw_ + par_.yaw_spinning_dyaw * par_.dc;
      next_goal.dyaw = par_.yaw_spinning_dyaw;
      previous_yaw_ = next_goal.yaw;
    }
    else
    {
      // If the local_plan is small just use the previous yaw with no dyaw
      if (local_plan.size() < 5)
      {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
      }
      else
      {
        getDesiredYaw(next_goal);
      }
    }

    if (par_.use_hardware && par_.provide_goal_in_global_frame)
    {
      next_goal.yaw -= yaw_init_offset_;
    }

    next_goal.dyaw = std::clamp(next_goal.dyaw, -par_.w_max, par_.w_max);
  }
  else
  {
    next_goal.yaw = previous_yaw_;
    next_goal.dyaw = 0.0;
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the desired yaw for the next goal.
 * @param state &next_goal: Next goal state to update with desired yaw.
 */
void DYNUS::getDesiredYaw(state &next_goal)
{

  double diff = 0.0;
  double desired_yaw = 0.0;

  // Get state
  state local_state;
  getState(local_state);

  // Get G_term
  mtx_G_term_.lock();
  state G_term = G_term_;
  mtx_G_term_.unlock();

  switch (drone_status_)
  {
  case DroneStatus::YAWING:
    desired_yaw = atan2(G_term.pos[1] - next_goal.pos[1], G_term.pos[0] - next_goal.pos[0]);
    diff = desired_yaw - local_state.yaw;
    // std::cout << "diff1= " << diff << std::endl;
    break;
  case DroneStatus::TRAVELING:
  case DroneStatus::GOAL_SEEN:
    desired_yaw = atan2(next_goal.pos[1] - local_state.pos.y(), next_goal.pos[0] - local_state.pos.x());
    diff = desired_yaw - local_state.yaw;
    next_goal.yaw = desired_yaw;
    break;
  case DroneStatus::GOAL_REACHED:
    next_goal.dyaw = 0.0;
    next_goal.yaw = previous_yaw_;
    return;
  }

  dynus_utils::angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  yaw(diff, next_goal);
}

// ----------------------------------------------------------------------------

void DYNUS::yaw(double diff, state &next_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * (copysign(1, diff) * par_.w_max) + par_.alpha_filter_dyaw * dyaw_filtered_;
  next_goal.dyaw = dyaw_filtered_;
  next_goal.yaw = previous_yaw_ + dyaw_filtered_ * par_.dc;
  previous_yaw_ = next_goal.yaw;
}

// ----------------------------------------------------------------------------

/**
 * @brief Starts exploration mode.
 */
void DYNUS::startExploration()
{

  mtx_G_.lock();
  mtx_state_.lock();
  G_ = state_;
  mtx_state_.unlock();
  mtx_G_.unlock();

  // Set the drone status to YAWING
  changeDroneStatus(DroneStatus::YAWING);

  // Set the terminal goal
  terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the terminal goal.
 * @param const state &term_goal: Desired terminal goal state.
 */
void DYNUS::setTerminalGoal(const state &term_goal)
{

  // std::cout << "terminal goal is set" << std::endl;

  if (par_.flight_mode == "exploration")
  {
    terminal_goal_initialized_ = true;
    changeDroneStatus(DroneStatus::TRAVELING);
    return;
  }

  if (par_.flight_mode == "object_trackign")
  {
    object_received_ = true;
  }

  // Get the state
  state local_state;
  getState(local_state);

  // Set the terminal goal
  setGterm(term_goal);

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = dynus_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
  mtx_G_.unlock();

  if (par_.use_initial_yawing)
  {
    changeDroneStatus(DroneStatus::YAWING);
  }
  else
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  if (!terminal_goal_initialized_)
    terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Changes the drone's status (YAWING, TRAVELING, GOAL_SEEN, GOAL_REACHED).
 * @param int new_status: New status value.
 */
void DYNUS::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
    return;

  std::cout << "Changing DroneStatus from ";

  switch (drone_status_)
  {
  case DroneStatus::YAWING:
    std::cout << bold << "status_=YAWING" << reset;
    break;
  case DroneStatus::TRAVELING:
    std::cout << bold << "status_=TRAVELING" << reset;
    break;
  case DroneStatus::GOAL_SEEN:
    std::cout << bold << "status_=GOAL_SEEN" << reset;
    break;
  case DroneStatus::GOAL_REACHED:
    std::cout << bold << "status_=GOAL_REACHED" << reset;
    break;
  }

  std::cout << " to ";

  switch (new_status)
  {
  case DroneStatus::YAWING:
    std::cout << bold << "status_=YAWING" << reset;
    break;
  case DroneStatus::TRAVELING:
    std::cout << bold << "status_=TRAVELING" << reset;
    break;
  case DroneStatus::GOAL_SEEN:
    std::cout << bold << "status_=GOAL_SEEN" << reset;
    break;
  case DroneStatus::GOAL_REACHED:
    std::cout << bold << "status_=GOAL_REACHED" << reset;
    break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if all necessary components are initialized.
 * @return bool
 */
bool DYNUS::checkReadyToReplan()
{

  if (!kdtree_map_initialized_ ||
      !kdtree_unk_initialized_ ||
      !state_initialized_ ||
      !terminal_goal_initialized_ ||
      !dgp_manager_.isMapInitialized() ||
      // !map_size_initialized_ ||
      (par_.use_frontiers && !frontier_initialized_) ||
      (par_.flight_mode == "object_tracking" && !object_received_))
  {
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if all necessary components are initialized for updating the map.
 * @return bool
 */
bool DYNUS::checkReadyToUpdateMap()
{

  if (!state_initialized_ || !terminal_goal_initialized_ || !dgp_manager_.isMapInitialized())
  {
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_unk)
{
  // 1) Atomically store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    pclptr_map_ = pclptr_map;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
    pclptr_unk_ = pclptr_unk;
  }

  // Update the map size
  state local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // 2) Octomap update (unlocked)
  dgp_manager_.updateMap(wdx_, wdy_, wdz_, map_center_, pclptr_map_);

  // 3) Known‐space KD‐tree
  if (pclptr_map_ && !pclptr_map_->points.empty())
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = true;
    dgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
  }
  else
  {
    RCLCPP_WARN(
        rclcpp::get_logger("dynus"),
        "updateMap: member pclptr_map_ was null or empty; skipping KD‐tree update");
  }

  // 4) Unknown‐space KD‐tree
  if (pclptr_unk_ && !pclptr_unk_->points.empty())
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
    kdtree_unk_.setInputCloud(pclptr_unk_);
    kdtree_unk_initialized_ = true;
    // merge known into unknown vector
    dgp_manager_.updateVecUnknownOccupied(pclptr_to_vec(pclptr_unk_));
    dgp_manager_.insertVecOccupiedToVecUnknownOccupied();
  }
  else
  {
    RCLCPP_WARN(
        rclcpp::get_logger("dynus"),
        "updateMap: member pclptr_unk_ was null or empty; skipping KD‐tree update");
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Finds frontiers for exploration.
 * @param Eigen::Vector3d &best_frontier: Output best frontier position.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @return bool
 */
bool DYNUS::findFrontiers(Eigen::Vector3d &best_frontier, const Eigen::Matrix4d &camera_transform)
{

  if (par_.flight_mode == "terminal_goal" || !state_initialized_ || !terminal_goal_initialized_ || !dgp_manager_.isMapInitialized() || !map_size_initialized_ || !octree_initialized_)
    return false;

  // Initialization
  std::shared_ptr<octomap::TimedOcTree> d435_octree;
  std::shared_ptr<octomap::TimedOcTree> mid360_octree;

  MyTimer timer(true);

  // Extract the frontiers
  bool result = extractFrontierPoints(best_frontier, d435_octree_, mid360_octree_, camera_transform);

  // Print the time
  timer.printMs("findFrontiers");

  if (!frontier_initialized_ && result)
    frontier_initialized_ = true;

  return result;
}

// ----------------------------------------------------------------------------

/**
 * @brief Extracts frontier points.
 * @ref https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d/blob/master/include/exploration.h
 * @param Eigen::Vector3d &best_frontier: Output best frontier position.
 * @param const std::shared_ptr<octomap::TimedOcTree> &d435_octree_ptr: Pointer to d435 octree.
 * @param const std::shared_ptr<octomap::TimedOcTree> &mid360_octree_ptr: Pointer to mid360 octree.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @return bool
 *
 */
bool DYNUS::extractFrontierPoints(Eigen::Vector3d &best_frontier, const std::shared_ptr<octomap::TimedOcTree> &d435_octree_ptr, const std::shared_ptr<octomap::TimedOcTree> &mid360_octree_ptr, const Eigen::Matrix4d &camera_transform)
{

  // Initialize the frontier points from both octrees
  vec_Vecf<3> d435_frontier_points;
  vec_Vecf<3> d435_frontier_directions;
  vec_Vecf<3> mid360_frontier_points;
  vec_Vecf<3> mid360_frontier_directions;

  computeAllFrontierPoints(d435_octree_ptr, d435_frontier_points, d435_frontier_directions);
  if (!par_.use_only_d435_for_frontiers)
    computeAllFrontierPoints(mid360_octree_ptr, mid360_frontier_points, mid360_frontier_directions);

  // Mid360's frontiers are on the floor and want to clear them using D435's octree
  if (!par_.use_only_d435_for_frontiers)
    filterFrontiers(d435_octree_ptr, mid360_frontier_points);

  // Sum the frontier points
  vec_Vecf<3> frontier_points;
  frontier_points.insert(frontier_points.end(), d435_frontier_points.begin(), d435_frontier_points.end());
  if (!par_.use_only_d435_for_frontiers)
    frontier_points.insert(frontier_points.end(), mid360_frontier_points.begin(), mid360_frontier_points.end());

  // Sum the frontier directions
  vec_Vecf<3> frontier_directions;
  frontier_directions.insert(frontier_directions.end(), d435_frontier_directions.begin(), d435_frontier_directions.end());
  if (!par_.use_only_d435_for_frontiers)
    frontier_directions.insert(frontier_directions.end(), mid360_frontier_directions.begin(), mid360_frontier_directions.end());

  if (frontier_points.empty())
  {
    std::cout << "No frontiers found" << std::endl;
    return false;
  }

  // TODO: Find best frontiers with increased search radius (we first start with small search radius, but if no frontiers are found, we increase the search radius)
  computeBestFrontiers(frontier_points, camera_transform, frontier_directions);

  mtx_best_frontier_.lock();
  best_frontier = best_frontier_.pos;
  mtx_best_frontier_.unlock();

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes all frontier points from an octree.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to input octree.
 * @param vec_Vecf<3> &frontier_points: Output frontier point coordinates.
 * @param vec_Vecf<3> &frontier_directions: Output frontier directions.
 */
void DYNUS::computeAllFrontierPoints(const std::shared_ptr<octomap::TimedOcTree> &octree,
                                     vec_Vecf<3> &frontier_points,
                                     vec_Vecf<3> &frontier_directions)
{
  // Acquire the current state once.
  mtx_state_.lock();
  state local_state = state_;
  mtx_state_.unlock();

  // Define the bounding box based on the state and parameters.
  octomap::point3d min_point(local_state.pos[0] - par_.depth_camera_depth_max - par_.frontier_search_buffer,
                             local_state.pos[1] - par_.depth_camera_depth_max - par_.frontier_search_buffer,
                             local_state.pos[2] - par_.depth_camera_depth_max - par_.frontier_search_buffer);
  octomap::point3d max_point(local_state.pos[0] + par_.depth_camera_depth_max + par_.frontier_search_buffer,
                             local_state.pos[1] + par_.depth_camera_depth_max + par_.frontier_search_buffer,
                             local_state.pos[2] + par_.depth_camera_depth_max + par_.frontier_search_buffer);

  // Get leaf box
  auto begin_it = octree->begin_leafs_bbx(min_point, max_point);
  auto end_it = octree->end_leafs_bbx();

  // First pass: collect all free leaf nodes within the bounding box.
  std::vector<OctreeLeafData> leaf_data;
  for (auto it = begin_it; it != end_it; ++it)
  {
    // Only consider free voxels.
    if (octree->isNodeOccupied(*it))
      continue;

    OctreeLeafData data;
    data.x = it.getX();
    data.y = it.getY();
    data.z = it.getZ();
    data.center = it.getCoordinate();
    data.size = it.getSize();
    leaf_data.push_back(data);
  }

  // Prepare global containers for frontier results.
  vec_Vec3f frontier_points_local;
  vec_Vec3f frontier_directions_local;

// Process each leaf node in parallel.
#pragma omp parallel
  {
    vec_Vec3f thread_points;
    vec_Vec3f thread_directions;

#pragma omp for nowait
    for (size_t i = 0; i < leaf_data.size(); i++)
    {
      const auto &data = leaf_data[i];
      double x_cur = data.x;
      double y_cur = data.y;
      double z_cur = data.z;
      const octomap::point3d &voxel_center = data.center;
      double node_size = data.size;
      double box_res = node_size / 2.0 + par_.octomap_res;

      bool isOccupiedNeighbor = false;
      bool isFrontier = false;

      int num_known_free = 0;
      octomap::point3d known_free_voxel(0.0, 0.0, 0.0);
      int num_unknown = 0;
      octomap::point3d unknown_voxel(0.0, 0.0, 0.0);

      // Examine neighbors in the XY plane (z fixed).
      for (double x_buf = x_cur - box_res; x_buf < x_cur + box_res; x_buf += par_.octomap_res)
      {
        for (double y_buf = y_cur - box_res; y_buf < y_cur + box_res; y_buf += par_.octomap_res)
        {
          octomap::point3d neighbor_center = voxel_center +
                                             octomap::point3d(x_buf - x_cur, y_buf - y_cur, 0.0);
          octomap::TimedOcTreeNode *neighbor_node = octree->search(octomap::point3d(x_buf, y_buf, z_cur));

          if (neighbor_node && !octree->isNodeOccupied(neighbor_node))
          {
            known_free_voxel = known_free_voxel + neighbor_center;
            num_known_free++;
          }
          if (!neighbor_node)
          {
            unknown_voxel = unknown_voxel + neighbor_center;
            num_unknown++;
          }
          if (neighbor_node && octree->isNodeOccupied(neighbor_node))
          {
            isOccupiedNeighbor = true;
            break;
          }
        }
        if (isOccupiedNeighbor)
          break;
      }

      if (num_known_free > par_.frontier_min_known_free_thresh &&
          num_unknown > par_.frontier_min_unknown_thresh)
      {
        isFrontier = true;
      }

      if (!isOccupiedNeighbor && isFrontier)
      {
        // Compute the average positions for known free and unknown voxels.
        Eigen::Vector3d known_free_avg(known_free_voxel.x(), known_free_voxel.y(), known_free_voxel.z());
        known_free_avg /= num_known_free;
        Eigen::Vector3d unknown_avg(unknown_voxel.x(), unknown_voxel.y(), unknown_voxel.z());
        unknown_avg /= num_unknown;

        Eigen::Vector3d direction = unknown_avg - known_free_avg;
        // Skip if the computed direction is degenerate.
        if (direction[0] == 0 && direction[1] == 0)
          continue;
        direction.normalize();

        thread_directions.push_back(direction);
        thread_points.push_back(Eigen::Vector3d(voxel_center.x(), voxel_center.y(), voxel_center.z()));

        // Optionally, add additional neighboring points if the node size is larger.
        if (node_size > par_.octomap_res)
        {
          for (double x_buf = x_cur - box_res; x_buf < x_cur + box_res; x_buf += box_res)
          {
            for (double y_buf = y_cur - box_res; y_buf < y_cur + box_res; y_buf += box_res)
            {
              for (double z_buf = z_cur - box_res; z_buf < z_cur + box_res; z_buf += box_res)
              {
                thread_points.push_back(Eigen::Vector3d(x_buf, y_buf, z_buf));
              }
            }
          }
        }
      }
    } // end for

// Merge the thread-local frontier data into the global containers.
#pragma omp critical
    {
      frontier_points_local.insert(frontier_points_local.end(), thread_points.begin(), thread_points.end());
      frontier_directions_local.insert(frontier_directions_local.end(),
                                       thread_directions.begin(), thread_directions.end());
    }
  } // end parallel region

  // Update the output containers.
  frontier_points = frontier_points_local;
  frontier_directions = frontier_directions_local;
}

// ----------------------------------------------------------------------------

/**
 * @brief Filters frontier points.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to octree.
 * @param vec_Vecf<3> &frontiers: Frontier points to filter.
 */
void DYNUS::filterFrontiers(const std::shared_ptr<octomap::TimedOcTree> &octree_ptr, vec_Vecf<3> &frontiers)
{

  // Filter the frontiers
  vec_Vecf<3> filtered_frontiers;
  for (const auto &frontier : frontiers)
  {
    // Check if the frontier is in free space of the octree
    octomap::point3d frontier_point(frontier[0], frontier[1], frontier[2]);
    auto *node = octree_ptr->search(frontier_point);

    if (!node) // Unknown voxel
    {
      filtered_frontiers.push_back(frontier);
    }
  }

  frontiers = filtered_frontiers;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the mean frontier.
 * @param const vec_Vecf<3> &visible_frontiers: Input visible frontier points.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to octree.
 */
void DYNUS::findMeanFrontier(const vec_Vecf<3> &visible_frontiers, const std::shared_ptr<octomap::TimedOcTree> &octree_ptr)
{
  if (visible_frontiers.empty())
  {
    // if visible_frontiers is empty, keep the best_frontier as it is
    return;
  }

  // Step 1: Compute the mean frontier
  Eigen::Vector3d mean_frontier = Eigen::Vector3d::Zero();
  for (const auto &frontier : visible_frontiers)
  {
    mean_frontier += frontier;
  }
  mean_frontier /= visible_frontiers.size();

  // Step 2: Find the closest frontier to the mean frontier
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d closest_frontier = mean_frontier;

  for (const auto &frontier : visible_frontiers)
  {
    double dist = (frontier - mean_frontier).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_frontier = frontier;
    }
  }

  mean_frontier = closest_frontier; // Update the mean frontier to the closest

  // Step 3: Find unknown neighbors of the mean frontier
  Eigen::Vector3d mean_unknown_neighbors = Eigen::Vector3d::Zero();
  int unknown_count = 0;

  double voxel_size = octree_ptr->getResolution();
  for (double dx = -voxel_size; dx <= voxel_size; dx += voxel_size)
  {
    for (double dy = -voxel_size; dy <= voxel_size; dy += voxel_size)
    {
      for (double dz = -voxel_size; dz <= voxel_size; dz += voxel_size)
      {
        // Skip the center voxel
        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        // Neighbor voxel position
        octomap::point3d neighbor_pos(mean_frontier[0] + dx,
                                      mean_frontier[1] + dy,
                                      mean_frontier[2] + dz);

        // Check if the neighbor is unknown
        auto *node = octree_ptr->search(neighbor_pos);
        if (!node) // Unknown voxel
        {
          mean_unknown_neighbors += Eigen::Vector3d(neighbor_pos.x(), neighbor_pos.y(), neighbor_pos.z());
          unknown_count++;
        }
      }
    }
  }

  if (!is_best_frontier_initialized_)
  {
    mtx_best_frontier_.lock();
    best_frontier_.pos = mean_frontier;
    mtx_best_frontier_.unlock();
  }

  if (unknown_count > 0)
  {
    mean_unknown_neighbors /= unknown_count;

    // Step 4: Compute the direction from mean_frontier to mean_unknown_neighbors
    Eigen::Vector3d direction_to_unknown = (mean_unknown_neighbors - mean_frontier).normalized();

    // Debug output
    std::cout << "Direction to unknown: " << direction_to_unknown.transpose() << std::endl;

    // Step 5: Update the best frontier
    // TODO: make sure the last yaw is used in yaw optimization
    mtx_best_frontier_.lock();
    best_frontier_.pos = par_.frontier_update_alpha * mean_frontier + (1 - par_.frontier_update_alpha) * best_frontier_.pos;
    best_frontier_.yaw = atan2(mean_unknown_neighbors[1] - mean_frontier[1], mean_unknown_neighbors[0] - mean_frontier[0]);
    mtx_best_frontier_.unlock();
  }
  else
  {
    std::cout << "No unknown neighbors found around mean_frontier." << std::endl;
    // Step 5: Update the best frontier
    mtx_best_frontier_.lock();
    best_frontier_.pos = par_.frontier_update_alpha * mean_frontier + (1 - par_.frontier_update_alpha) * best_frontier_.pos;
    mtx_best_frontier_.unlock();
  }

  if (!is_best_frontier_initialized_)
  {
    is_best_frontier_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the best frontier.
 * @param const vec_Vecf<3> &global_frontiers: Input candidate frontier points.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @param vec_Vecf<3> &frontier_directions: Output frontier directions.
 */
void DYNUS::computeBestFrontiers(const vec_Vecf<3> &global_frontiers,
                                 const Eigen::Matrix4d &camera_transform,
                                 vec_Vecf<3> &frontier_directions)
{
  // Get local state (protected by mutex)
  mtx_state_.lock();
  state state_local = state_;
  mtx_state_.unlock();

  // Get previous best frontier (protected by mutex)
  state previous_best_frontier;
  mtx_best_frontier_.lock();
  previous_best_frontier = best_frontier_;
  mtx_best_frontier_.unlock();

  // Global reduction variables
  double global_min_cost = std::numeric_limits<double>::max();
  int global_best_frontier_idx = -1;
  vec_Vecf<3> combined_frontier_points;

// Parallel region: each thread computes its own best cost and collects valid frontiers.
#pragma omp parallel
  {
    double local_min_cost = std::numeric_limits<double>::max();
    int local_best_frontier_idx = -1;
    vec_Vecf<3> local_frontier_points;

#pragma omp for nowait
    for (int idx = 0; idx < static_cast<int>(global_frontiers.size()); idx++)
    {
      Eigen::Vector3d frontier = global_frontiers[idx];

      // Filter out NaNs
      if (std::isnan(frontier.x()) || std::isnan(frontier.y()) || std::isnan(frontier.z()))
        continue;

      // Filter based on z-range
      if (frontier.z() < par_.z_min || frontier.z() > par_.z_max)
        continue;

      // Filter based on local sliding window map
      if (!checkPointWithinMap(frontier))
        continue;

      // Filter points too close to camera
      if (dynus_utils::euclideanDistance(frontier, state_local.pos) < par_.min_dist_from_frontier_to_camera)
        continue;

      // Transform the point to camera frame
      Eigen::Vector4d point_world_homo(frontier.x(), frontier.y(), frontier.z(), 1.0);
      Eigen::Vector4d point_camera_homo = camera_transform * point_world_homo;

      // Depth-based filter
      if (par_.d435_depth_min > point_camera_homo(2))
        continue;

      // Height difference filter
      if (std::abs(frontier.z() - state_local.pos.z()) > par_.max_z_diff_from_frontier_to_camera)
        continue;

      // Save this valid frontier point in the thread-local vector.
      local_frontier_points.push_back(frontier);

      // (1) Compute desired velocity cost
      Eigen::Vector3d desired_velocity = (frontier - state_local.pos) * (v_max_ / par_.depth_camera_depth_max);
      double desired_velocity_norm = desired_velocity.norm();
      double desired_velocity_cost = 1.0 / (desired_velocity_norm + 0.0001);

      // (2) Compute cost based on distance from camera's z-axis
      double dist_from_z_axis = std::sqrt(point_camera_homo(0) * point_camera_homo(0) +
                                          point_camera_homo(1) * point_camera_homo(1));

      // (3) Compute cost based on closeness to the previous best frontier (only XY distance)
      double dist_to_prev_best_frontier = std::sqrt(
          (previous_best_frontier.pos(0) - state_local.pos(0)) * (previous_best_frontier.pos(0) - state_local.pos(0)) +
          (previous_best_frontier.pos(1) - state_local.pos(1)) * (previous_best_frontier.pos(1) - state_local.pos(1)));

      // (4) Encourage a positive z-axis in camera direction
      double positive_z_camera = -point_camera_homo(2);

      // (5) Compute goal proximity (if flight_mode is terminal_goal)
      double goal_proximity = 0.0;
      if (par_.flight_mode == "terminal_goal")
      {
        state local_G;
        getG(local_G);
        goal_proximity = (frontier - local_G.pos).norm();
      }

      // (6) Compute information gain cost by counting neighboring frontiers
      int num_neighbor_frontier = 0;
      for (const auto &sub_frontier : global_frontiers)
      {
        if ((frontier - sub_frontier).norm() < par_.frontier_neighbor_thresh_for_info_gain)
        {
          num_neighbor_frontier++;
        }
      }
      double info_gain_cost = global_frontiers.size() - num_neighbor_frontier;

      // Total cost (weighted sum)
      double total_cost = par_.desired_velocity_cost_weight * desired_velocity_cost +
                          par_.dist_from_z_axis_weight * dist_from_z_axis +
                          par_.dist_to_prev_best_frontier_weight * dist_to_prev_best_frontier +
                          par_.positive_z_camera_weight * positive_z_camera +
                          par_.goal_proximity_weight * goal_proximity +
                          par_.info_gain_cost_weight * info_gain_cost;

      // Update the local best candidate if a lower cost is found.
      if (total_cost < local_min_cost)
      {
        local_min_cost = total_cost;
        local_best_frontier_idx = idx;
      }
    } // end for loop

// Merge thread-local results into global variables
#pragma omp critical
    {
      // Append the thread's frontier points to the combined vector.
      combined_frontier_points.insert(combined_frontier_points.end(),
                                      local_frontier_points.begin(),
                                      local_frontier_points.end());
      // Update global best candidate if this thread found a lower cost.
      if (local_min_cost < global_min_cost)
      {
        global_min_cost = local_min_cost;
        global_best_frontier_idx = local_best_frontier_idx;
      }
    }
  } // end parallel region

  // If no frontier passed the filters, then exit.
  if (combined_frontier_points.empty())
  {
    std::cout << "No frontiers found" << std::endl;
    return;
  }

  // Update the shared frontier_points_ with locking.
  mtx_frontier_points_.lock();
  frontier_points_ = combined_frontier_points;
  mtx_frontier_points_.unlock();

  // Update the best frontier's position using the global best candidate.
  state local_best_frontier;
  local_best_frontier.pos = global_frontiers[global_best_frontier_idx];

  // Compute yaw based on flight mode.
  state local_G_term;
  getGterm(local_G_term);

  if (par_.flight_mode == "terminal_goal")
  {
    // Direct the best frontier toward the terminal goal.
    local_best_frontier.yaw = std::atan2(local_G_term.pos[1] - local_best_frontier.pos[1],
                                         local_G_term.pos[0] - local_best_frontier.pos[0]);
  }
  else if (par_.flight_mode == "exploration")
  {
    // Use the average direction from the provided frontier directions.
    Eigen::Vector3d mean_direction = Eigen::Vector3d::Zero();
    for (const auto &direction : frontier_directions)
    {
      mean_direction += direction;
    }
    mean_direction /= frontier_directions.size();
    local_best_frontier.yaw = std::atan2(mean_direction[1], mean_direction[0]);
  }

  // Update the best frontier in the shared variable with locking.
  mtx_best_frontier_.lock();
  best_frontier_ = local_best_frontier;
  mtx_best_frontier_.unlock();

  if (!is_best_frontier_initialized_)
    is_best_frontier_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets visible frontiers.
 * @param vec_Vecf<3> &visible_frontiers: Output visible frontier points.
 */
void DYNUS::getVisibleFrontiers(vec_Vecf<3> &visible_frontiers)
{
  visible_frontiers = visible_frontiers_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets all frontiers.
 * @param vec_Vecf<3> &frontiers: Output all frontier points.
 */
void DYNUS::getAllFrontiers(vec_Vecf<3> &frontiers)
{
  mtx_frontier_points_.lock();
  frontiers = frontier_points_;
  mtx_frontier_points_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Set the initial pose.
 * @param const geometry_msgs::msg::TransformStamped &init_pose: Initial pose.
 */
void DYNUS::setInitialPose(const geometry_msgs::msg::TransformStamped &init_pose)
{
  init_pose_ = init_pose;

  // First compute transformation matrix from init_pose_ (geometry_msgs::msg::TransformStamped)
  Eigen::Matrix4d init_pose_transform = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond init_pose_quat(init_pose_.transform.rotation.w, init_pose_.transform.rotation.x, init_pose_.transform.rotation.y, init_pose_.transform.rotation.z);
  Eigen::Vector3d init_pose_translation(init_pose_.transform.translation.x, init_pose_.transform.translation.y, init_pose_.transform.translation.z);
  init_pose_transform.block<3, 3>(0, 0) = init_pose_quat.toRotationMatrix();
  init_pose_transform.block<3, 1>(0, 3) = init_pose_translation;

  // Get initial pose
  init_pose_transform_ = init_pose_transform;
  init_pose_transform_rotation_ = init_pose_quat.toRotationMatrix();
  yaw_init_offset_ = std::atan2(init_pose_transform_rotation_(1, 0),
                                init_pose_transform_rotation_(0, 0));

  std::cout << bold << green << "yaw_init_offset_: " << yaw_init_offset_ << reset << std::endl;

  // Get the inverse of init_pose_ (geometry_msgs::msg::TransformStamped)
  init_pose_transform_inv_ = init_pose_transform.inverse();
  init_pose_transform_rotation_inv_ = init_pose_quat.toRotationMatrix().inverse();
  // yaw_init_offset_ = std::atan2(init_pose_transform_rotation_inv_(1, 0),
  // init_pose_transform_rotation_inv_(0, 0));
}

// ----------------------------------------------------------------------------

// Apply the initial pose transformation to the pwp
void DYNUS::applyInitiPoseTransform(PieceWisePol &pwp)
{
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++)
  {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++)
    {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_ * coeff;

      // cout agent frame pose
      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

// Apply the inverse of initial pose transformation to the pwp
void DYNUS::applyInitiPoseInverseTransform(PieceWisePol &pwp)
{
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++)
  {

    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++)
    {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_inv_ * coeff;

      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the best frontier.
 * @param state &best_frontier: Output best frontier state.
 */
void DYNUS::getBestFrontier(state &best_frontier)
{
  if (is_best_frontier_initialized_)
  {

    mtx_state_.lock();
    state state_local = state_;
    mtx_state_.unlock();

    mtx_best_frontier_.lock();
    state local_best_frontier = best_frontier_;
    mtx_best_frontier_.unlock();

    // Find the point that is free and closest to the projected point
    // TODO: not sure if this is implemented correctly - vertical climb fails with this
    // dgp_manager_.findClosestFreePoint(best_frontier_.pos, best_frontier.pos);
    // std::cout << "actual best_frontier_: " << best_frontier_.pos.transpose() << std::endl;
    // std::cout << "projected best_frontier: " << best_frontier.pos.transpose() << std::endl;

    // TODO: projection not working well - just did hacky solution of making the map smaller
    best_frontier.pos = dynus_utils::projectPointToSphere(state_local.pos, local_best_frontier.pos, par_.horizon);
    best_frontier.yaw = local_best_frontier.yaw;
  }
  else
  {
    std::cout << "Best frontier not initialized" << std::endl;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if the drone is moving.
 * @return bool
 */
bool DYNUS::isMoving()
{
  if (drone_status_ == DroneStatus::TRAVELING || drone_status_ == DroneStatus::GOAL_SEEN)
  {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if there are nearby trajectories.
 * @param double current_time: Current timestamp.
 * @return bool
 */
bool DYNUS::checkNearbyTrajs(double current_time)
{

  // Get the state
  mtx_state_.lock();
  state state_local = state_;
  mtx_state_.unlock();

  // Get the trajs
  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  if (local_trajs.empty())
  {
    return false;
  }

  // Check if there's any nearby trajs
  for (const auto &traj : local_trajs)
  {
    if ((traj->eval(current_time) - state_local.pos).norm() < par_.min_dist_from_agent_to_traj)
    {
      return true;
    }
  }

  return false;
}

// ----------------------------------------------------------------------------

/**
 * @brief Processes a successful detection.
 * @param const std::string &detected_object: Identifier of the detected object.
 */
void DYNUS::detectSuccessfulDetection(const std::string &detected_object)
{
  std::cout << bold << green << "Detected object: " << detected_object << reset << std::endl;
  std::cout << "Changing the drone status to GOAL_REACHED" << std::endl;
  changeDroneStatus(DroneStatus::GOAL_REACHED);
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if the goal is reached.
 * @return bool
 */
bool DYNUS::goalReachedCheck()
{
  if (checkReadyToReplan() && drone_status_ == DroneStatus::GOAL_REACHED)
  {
    return true;
  }
  return false;
}