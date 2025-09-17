/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <cmath> // for cos, sin, M_PI
#include <algorithm>
#include <vector>
#include <stdlib.h>

#include "timer.hpp"
#include "dgp/termcolor.hpp"
#include "dynus/dynus_type.hpp"
#include <dynus/utils.hpp>
#include "dgp/dgp_manager.hpp"
#include "dynus/closed_form_solver.hpp"
#include <dynus/lbfgs_solver.hpp>
#include <dgp/TimedOcTree.h>
#include <dgp/TimedOcTreeNode.h>

#define MAP 1         // MAP refers to the occupancy grid
#define UNKNOWN_MAP 2 // UNKNOWN_MAP refers to the unkown grid

using namespace dynus;
using namespace termcolor;

// Type‐aliases
using Vec3   = Eigen::Vector3d;
using Vec3f  = Eigen::Matrix<double,3,1>;
using MatXd  = Eigen::MatrixXd;
using VecXd  = Eigen::VectorXd;

// ------------------------------------------
// 2) The aligned‐allocator for local blocks:
// ------------------------------------------
// your PlaneBlock is still:
using PlaneBlock = std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>,
                             Eigen::VectorXd>;

// replace your old ConstraintBlocks with:

// 1) an aligned‐allocator for a vector of PlaneBlock
using AlignedPlaneBlockVec = std::vector<
  PlaneBlock,
  Eigen::aligned_allocator<PlaneBlock>
>;

// 2) then your top‐level blocks is a vector of those
using ConstraintBlocks = std::vector<
  AlignedPlaneBlockVec,
  Eigen::aligned_allocator<AlignedPlaneBlockVec>
>;

enum DroneStatus
{
  YAWING = 0,
  TRAVELING = 1,
  GOAL_SEEN = 2,
  GOAL_REACHED = 3
};

struct OctreeLeafData
{
  double x;
  double y;
  double z;
  octomap::point3d center;
  double size;
};

class DYNUS
{

public:
  // Methods
  DYNUS(parameters par);                                                                                                           // Constructor
  bool needReplan(const state &local_state, const state &local_G_term, const state &last_plan_state);                              // Check if replan is needed
  bool findAandAtime(state &A, double &A_time, double current_time, double last_replaning_computation_time);                       // Find A and A time
  void filterStaticPushPoints(const Vec3f &latest_mean_push_point, const vec_Vecf<3> &global_path, bool found_static_push_points); // Filter static push points
  void staticPush(vec_Vecf<3> &global_path);                                                                                       // Static push
  bool checkIfPointOccupied(const Vec3f &point);
  bool checkIfPointFree(const Vec3f &point);
  bool checkIfPointWithinMap(const Vec3f &point);                                                                                                                      // Check if the point is occupied
  void dynamicPush(vec_Vecf<3> &global_path, double current_time, std::vector<std::shared_ptr<dynTraj>> &local_trajs);                                                 // Dynamic push
  bool getSafeCorridor(const vec_Vecf<3> &global_path, const state &A); // Get the safe corridor
  std::tuple<bool, bool> replan(double last_replaning_computation_time, double current_time);
  void initializeSolver(const std::shared_ptr<ClosedFromSolver> &traj_solver_ptr_);
  void startAdaptKValue();
  void getMap(vec_Vecf<3> &occupied_cells); // Publish the map
  void getFreeMap(vec_Vecf<3> &free_cells);
  void getGterm(state &G_term);                                                                                              // Get the terminal goal
  void setGterm(const state &G_term);
  void getG(state &G);       // Get the goal
  void getE(state &E);       // Get point E
  void setG(const state &G); // Set the goal
  void getA(state &A);
  void setA(const state &A); // Get A (starting point)
  void getA_time(double &A_time);
  void setA_time(double A_time);
  void getState(state &state);
  void getTrajs(std::vector<std::shared_ptr<dynTraj>> &out);            // Get the state
  void getLastPlanState(state &state);                                  // Get the last plan state
  void cleanUpOldTrajs(double current_time);                            // Clean up the old trajectories
  void addTraj(std::shared_ptr<dynTraj> new_traj, double current_time); // Add the trajectory
  std::vector<double> getDt(vec_Vecf<3> &global_path, const state &A);  // Get the time allocation
  void updateState(state data);                                         // Update the state
  bool getNextGoal(state &next_goal);                                   // Get the next goal
  void detectSuccessfulDetection(const std::string &detected_object);
  bool checkReadyToReplan();
  bool checkReadyToUpdateMap();                 // Check if all the things are initialized except the planner
  void setTerminalGoal(const state &term_goal); // Set the terminal goal
  void changeDroneStatus(int new_status);       // Change the drone status
  void getDesiredYaw(state &next_goal);
  void yaw(double diff, state &next_goal);                                                                                                                                                                                           // Get the desired yaw
  // void updateMapCallback(double octmap_received_time, const std::shared_ptr<octomap::TimedOcTree> &lidar_octree, const std::shared_ptr<octomap::TimedOcTree> &depth_camera_octree);                                                  // Update the map
  bool safetyCheck(PieceWisePol &pwp, double current_time);                                                                                                                                                                          // Safety check
  void computeG(const state &A, const state &G_term, double horizon);                                                                                                                                                                // Compute the goal
  void startExploration();                                                                                                                                                                                                           // Start the exploration
  void getAllFrontiers(vec_Vecf<3> &all_frontiers);                                                                                                                                                                                  // Get all the frontiers
  bool extractFrontierPoints(Eigen::Vector3d &best_frontier, const std::shared_ptr<octomap::TimedOcTree> &d435_octree_ptr, const std::shared_ptr<octomap::TimedOcTree> &mid360_octree_ptr, const Eigen::Matrix4d &camera_transform); // Extract the frontier points
  void computeAllFrontierPoints(const std::shared_ptr<octomap::TimedOcTree> &octree_ptr, vec_Vecf<3> &frontier_points, vec_Vecf<3> &frontier_directions);                                                                            // Compute all the frontier points
  void computeBestFrontiers(const vec_Vecf<3> &global_frontiers, const Eigen::Matrix4d &camera_transform, vec_Vecf<3> &frontier_directions);                                                                                         // Compute the best frontiers
  void getVisibleFrontiers(vec_Vecf<3> &visible_frontiers);                                                                                                                                                                          // Get the visible frontiers
  void findMeanFrontier(const vec_Vecf<3> &visible_frontiers, const std::shared_ptr<octomap::TimedOcTree> &octree_ptr);                                                                                                              // Find the mean frontier
  void getBestFrontier(state &best_frontier);                                                                                                                                                                                        // Get the best frontier
  void setD435CameraIntrinsics(const Eigen::Matrix3d &d435_intrinsics);                                                                                                                                                              // Set the D435 camera intrinsics
  bool findFrontiers(Eigen::Vector3d &best_frontier, const Eigen::Matrix4d &camera_transform);
  void filterFrontiers(const std::shared_ptr<octomap::TimedOcTree> &octree_ptr, vec_Vecf<3> &frontiers);    // Filter the frontiers
  bool checkNearbyTrajs(double current_time);                                                               // Check the nearby trajectories
  bool goalReachedCheck();                                                                                  // Check if the goal is reached
  bool isMoving();                                                                                          // Check if the drone is moving
  void checkFuturePlanSafety();                                                                             // Check if the plan is safe
  void switchToSafePath(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time); // Generate the contingency plan
  void generateContingencyPlan(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time);
  void computeMapSize(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos); // Compute the map size
  bool checkPointWithinMap(const Eigen::Vector3d &point) const;                                                        // Check if the point is within the map
  void updateMapRes();
  void getStaticPushPoints(vec_Vecf<3> &static_push_points);
  void getPpoints(vec_Vecf<3> &p_points);
  void getLocalGlobalPath(vec_Vecf<3> &local_global_path, vec_Vecf<3> &local_global_path_after_push);
  void getGlobalPath(vec_Vecf<3> &global_path);
  void getOriginalGlobalPath(vec_Vecf<3> &original_global_path); // For visualization
  void getFreeGlobalPath(vec_Vecf<3> &free_global_path);
  bool generateLocalTrajectory(const state &local_A,double A_time,vec_Vec3f &global_path,double &initial_guess_computation_time,double &local_traj_computation_time,std::shared_ptr<lbfgs::SolverLBFGS> &whole_traj_solver_ptr);
  void getCurrentWds(double &wdx, double &wdy, double &wdz);
  void resetData();
  void retrieveData(double &final_g,
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
                    double &yaw_fitting_time);
  void retrievePolytopes(vec_E<Polyhedron<3>> &poly_out_whole, vec_E<Polyhedron<3>> &poly_out_safe);
  void retrieveGoalSetpoints(std::vector<state> &goal_setpoints);
  void retrieveListSubOptGoalSetpoints(std::vector<std::vector<state>> &list_subopt_goal_setpoints);
  void retrievePwpToShare(PieceWisePol &pwp_to_share);
  void retrieveYawData(std::vector<double> &optimal_yaw_sequence,
                       std::vector<double> &yaw_control_points,
                       std::vector<double> &yaw_knots);
  void retrieveCPs(std::vector<Eigen::Matrix<double, 3, 6>> &cps);
  void setObjectTrackingPosition(const Eigen::Vector3d &object_pos); // Set the object tracking position

  // Safe path related
  bool computeSafePaths(const std::vector<state> &goal_setpoints, std::map<int, std::vector<state>> &safe_paths);                                                                          // Compute the safe paths
  bool generateSafePathPointsandincludeBestSafePath(std::vector<state> &goal_setpoints, std::vector<std::vector<state>> &safe_path_points, std::map<int, std::vector<state>> &safe_paths); // Generate the safe path points and include the best safe path
  bool generateSafePathsWithClosedForm(const state &start_goal_setpoint, std::vector<state> &safe_path, const std::shared_ptr<ClosedFromSolver> &safe_traj_solver_ptr);                    // Generate the safe paths with closed form
  void generatePotentialGoal(const state &start_goal_setpoint, state &potential_goal, double &closed_form_dt);                                                                             // Generate the potential goal
  double computeMinimumTimeUntilStop(double v0, double a0, double j);
  int findHPointIndex(const std::vector<state> &goal_setpoints);
  bool solveClosedForm(const state &start, const state &goal, std::vector<state> &safe_path, const std::shared_ptr<ClosedFromSolver> &safe_traj_solver_ptr, double closed_form_dt);
  bool generateGlobalPath(vec_Vecf<3> &global_path, double current_time, double last_replaning_computation_time);
  bool pushPath(vec_Vecf<3> &global_path, vec_Vecf<3> &free_global_path, double current_time);
  bool planLocalTrajectory(vec_Vecf<3> &global_path);
  bool planSafePaths(std::vector<std::vector<state>> &safe_path_points);
  bool appendToPlan();

  // For hardware experiments' initial pose
  void setInitialPose(const geometry_msgs::msg::TransformStamped &init_pose);
  void applyInitiPoseTransform(PieceWisePol &pwp);
  void applyInitiPoseInverseTransform(PieceWisePol &pwp);

  void updateMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_map, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pclptr_unk);
  void trimPathAtFirstMapCollision(vec_Vecf<3> &input_path);

private:
  // Parameters
  parameters par_;                                                       // Parameters of the planner
  DGPManager dgp_manager_;                                               // DGP Manager
  std::vector<LinearConstraint3D> safe_corridor_polytopes_whole_;        // Polytope (Linear) constraints for whole trajectory
  std::vector<LinearConstraint3D> safe_corridor_polytopes_safe_;         // Polytope (Linear) constraints for safe trajectories
  std::shared_ptr<lbfgs::SolverLBFGS> whole_traj_solver_ptr_;            // L-BFGS solver pointer for the whole trajectory
  std::vector<std::shared_ptr<ClosedFromSolver>> safe_traj_solver_ptrs_; // closed-form solver pointer for the safe trajectory
  std::shared_ptr<ClosedFromSolver> contingency_traj_solver_ptr_;        // closed-form solver pointer for the contingency trajectory
  std::vector<std::shared_ptr<dynTraj>> trajs_;                          // Dynamic trajectory
  Eigen::Vector3d v_max_3d_;                                             // Maximum velocity
  Eigen::Vector3d a_max_3d_;                                             // Maximum acceleration
  Eigen::Vector3d j_max_3d_;                                             // Maximum jerk
  double v_max_;
  double max_dist_vertexes_;                                                   // Maximum velocity
  lbfgs::planner_params_t planner_params_;                               // Planner parameters
  lbfgs::lbfgs_parameter_t lbfgs_params_;                                // L-BFGS parameters

  // Flags
  bool state_initialized_ = false;                           // State initialized
  bool terminal_goal_initialized_ = false;                   // Terminal goal initialized
  bool is_during_check_ = false;                             // During check
  bool use_adapt_k_value_ = false;                           // Use adapt k value
  bool look_at_direction_of_motion_ = true;                  // Look at the direction of motion
  bool is_best_frontier_initialized_ = false;                // Best frontier initialized
  bool use_frontiers_as_G_ = true;                           // Use frontiers as goal
  bool frontier_initialized_ = false;                        // Frontier initialized
  bool need_to_generate_safe_corridor_for_safe_path_ = true; // Need to generate safe corridor for safe path
  bool octree_initialized_ = false;                          // Octree initialized
  bool kdtree_map_initialized_ = false;                      // Kd-tree for the map initialized
  bool kdtree_unk_initialized_ = false;                      // Kd-tree for the map initialized

  // Data
  double final_g_ = 0.0;
  double global_planning_time_ = 0.0;
  double dgp_static_jps_time_ = 0.0;
  double dgp_check_path_time_ = 0.0;
  double dgp_dynamic_astar_time_ = 0.0;
  double dgp_recover_path_time_ = 0.0;
  double cvx_decomp_time_ = 0.0;
  double initial_guess_computation_time_ = 0.0;
  double local_traj_computation_time_ = 0.0;
  double safe_paths_time_ = 0.0;
  double safety_check_time_ = 0.0;
  double yaw_sequence_time_ = 0.0;
  double yaw_fitting_time_ = 0.0;
  vec_E<Polyhedron<3>> poly_out_whole_;
  vec_E<Polyhedron<3>> poly_out_safe_;
  std::vector<state> goal_setpoints_;
  // PieceWiseQuinticPol pwp_to_share_;
  std::vector<double> optimal_yaw_sequence_;
  std::vector<double> yaw_control_points_;
  std::vector<double> yaw_knots_;
  std::vector<Eigen::Matrix<double, 3, 6>> cps_;
  std::vector<Eigen::VectorXd> list_z_subopt_; // Store suboptimal solutions for visualization
  std::vector<std::vector<Eigen::Vector3d>> list_initial_guess_wps_subopt_;
  std::vector<std::vector<state>> list_subopt_goal_setpoints_;

  // Basis
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_;
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_inverse_;

  // Replanning-related variables
  state state_;                                    // State for the drone
  state G_;                                        // This goal is always inside of the map
  state A_;                                        // Starting point of the drone
  double A_time_;                                  // Time of the starting point
  state E_;                                        // The goal point of actual trajectory
  state G_term_;                                   // Terminal goal
  std::deque<state> plan_;                         // Plan for the drone
  std::deque<std::vector<state>> plan_safe_paths_; // Indicate if the state has a safe path
  double previous_yaw_ = 0.0;                      // Previous yaw
  double prev_dyaw_ = 0.0;                         // Previous dyaw
  double dyaw_filtered_ = 0.0;                     // Filtered dyaw

  // Drone status
  int drone_status_ = DroneStatus::GOAL_REACHED; // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED

  // Octree
  std::shared_ptr<octomap::TimedOcTree> d435_octree_;   // Octree for D435
  std::shared_ptr<octomap::TimedOcTree> mid360_octree_; // Octree for Mid360

  // Mutex
  std::mutex mtx_plan_;            // Mutex for the plan_
  std::mutex mtx_plan_safe_paths_; // Mutex for plan_safe_paths_
  std::mutex mtx_state_;           // Mutex for the state_
  std::mutex mtx_G_;               // Mutex for the G_
  std::mutex mtx_A_;               // Mutex for the A_
  std::mutex mtx_A_time_;          // Mutex for the A_time_
  std::mutex mtx_G_term_;          // Mutex for the G_term_
  std::mutex mtx_E_;               // Mutex for the E_
  std::mutex mtx_trajs_;           // Mutex for the trajs_
  std::mutex mtx_d435_octree_;     // Mutex for the d435_octree_
  std::mutex mtx_mid360_octree_;   // Mutex for the mid360_octree_
  std::mutex mtx_best_frontier_;   // Mutex for the best_frontier_
  std::mutex mtx_frontier_points_; // Mutex for the frontiers_
  std::mutex mtx_solve_dgp_;       // Mutex for the solveDGP
  std::mutex mtx_global_path_;     // Mutex for the global_path_
  std::mutex mtx_original_global_path_; // Mutex for the original_global_path_
  std::mutex mtx_kdtree_map_;       // Mutex for the map_
  std::mutex mtx_kdtree_unk_;           // Mutex for the unknown map_
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclptr_map_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclptr_unk_;

  // Map resolution
  double map_res_;

  // Counter for replanning failure
  int replanning_failure_count_ = 0; // Replanning failure count

  // Map size
  bool map_size_initialized_ = false; // Map size initialized
  int dgp_failure_count_ = 0;         // DGP failure count used for map size adaptation

  // Safe paths
  std::map<int, std::vector<state>> safe_paths_global_indexed_; // Safe paths - the int stores the id of the corresponding safe path, and the vector stores the safe paths starting from the starting point

  // Communication delay
  std::unordered_map<int, double> comm_delay_map_;

  // Dynamic k value
  int num_replanning_ = 0;                      // Number of replanning
  bool got_enough_replanning_ = false;          // Check if tot enough replanning
  int k_value_ = 0;                             // k value
  std::vector<double> store_computation_times_; // Store computation times
  double est_comp_time_ = 0.0;                  // Computation time estimation

  // Map size
  double wdx_, wdy_, wdz_;        // Width of the map
  Vec3f map_center_;              // Center of the map
  double current_dynamic_buffer_; // Current dynamic buffer

  // Global path
  vec_Vecf<3> global_path_;
  vec_Vecf<3> original_global_path_; // For visualization
  vec_Vecf<3> free_global_path_;

  // Static push
  vec_Vecf<3> static_push_points_;
  vec_Vecf<3> p_points_;
  vec_Vecf<3> local_global_path_;
  vec_Vecf<3> local_global_path_after_push_;

  // Frontiers
  vec_Vecf<3> visible_frontiers_;
  vec_Vecf<3> frontier_points_;
  state best_frontier_;
  int no_visible_frontiers_count_ = 0;

  // Initial pose
  geometry_msgs::msg::TransformStamped init_pose_;
  Eigen::Matrix4d init_pose_transform_;
  Eigen::Matrix3d init_pose_transform_rotation_;
  Eigen::Matrix4d init_pose_transform_inv_;
  Eigen::Matrix3d init_pose_transform_rotation_inv_;
  double yaw_init_offset_ = 0.0;

  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, 3>,
      Eigen::aligned_allocator<Eigen::Matrix<double, Eigen::Dynamic, 3>>>
      A_stat_;

  std::vector<
      Eigen::VectorXd,
      Eigen::aligned_allocator<Eigen::VectorXd>>
      b_stat_;

  // kd-tree for the map
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;       // kdtree of the point cloud of the occuppancy grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_unk_;       // kdtree of the point cloud of the unknown grid

  // Store data
  Eigen::VectorXd zopt_;
  double fopt_;

  // Object tracking
  bool object_received_ = false; // Object tracking received
  Eigen::Vector3d object_pos_;   // Object tracking position
};
