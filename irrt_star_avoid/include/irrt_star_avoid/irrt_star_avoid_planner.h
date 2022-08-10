#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <irrt_star_avoid/solvers/time_avoid_rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <irrt_star_avoid/time_avoid_metrics.h>
#include <graph_core/moveit_collision_checker.h>
#include <irrt_star_avoid/time_informed_sampler.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <graph_core/graph/graph_display.h>
#include <visualization_msgs/Marker.h>
#include <avoidance_intervals/avoidance_model.h>
#include <ros/ros.h>
#include <algorithm>

// #define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define COMMENT(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#include <fstream>
#include <iostream>
#include <memory>

namespace pathplan {
namespace irrt_star_avoid {

class IRRTStarAvoid: public planning_interface::PlanningContext
{
public:
  IRRTStarAvoid ( const std::string& name,
                const std::string& group,
                const moveit::core::RobotModelConstPtr& model
              );


  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);
  void updateAvoidModel(std::vector<Eigen::VectorXf> avoid_pts);
  virtual bool solve(planning_interface::MotionPlanResponse& res) override;
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).*/
  virtual bool terminate() override;

  /** \brief Clear the data structures used by the planner */
  virtual void clear() override;
  avoidance_intervals::modelPtr avoid_model_;
  void computeTransitions(double &tf, double &v0, double &vf, double &a1, double &a2, double v_max, double &t1, double &t2, bool recur);
  void time_parameterize(std::vector<Eigen::VectorXd> waypoints, std::vector<double> waypoint_times, std::vector<Eigen::VectorXd> &sequence, std::vector<Eigen::VectorXd> &seq_vels, std::vector<Eigen::VectorXd> &seq_accs, std::vector<double> &seq_times);
  
protected:
  // ros::NodeHandle m_nh;
  moveit::core::RobotModelConstPtr robot_model_;
  //planning_scene::PlanningSceneConstPtr pl
  ros::NodeHandle m_nh;
  // ros::AsyncSpinner spinner;
  ros::CallbackQueue m_queue;

  std::shared_ptr<pathplan::Display> display;

  ros::WallDuration m_max_planning_time;
  ros::WallDuration m_max_refining_time;

  unsigned int m_dof;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd m_lb;
  Eigen::VectorXd m_ub;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_accels_;
  double t_pad_ =0.0;
  std::string group_;
  bool m_tube_sampler;
  bool m_path_optimization;
  double m_forgetting_factor=0.99;
  double m_minimum_success_rate=1e-12;

  pathplan::TimeAvoidMetricsPtr metrics;
  pathplan::MetricsPtr base_metrics;
  pathplan::CollisionCheckerPtr checker;
  pathplan::ParallelRobotPointCloudsPtr point_cloud_checker;

  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_ud;

  double collision_distance=0.04;
  int collision_thread_=4;
  bool m_is_running=false;
  bool m_stop=false;
  double plot_interval_=5;
  bool display_flag=false;
  int max_iterations = 0;
  bool allow_goal_collision = false;
  double max_connection_cost;

private:
  ros::Publisher vis_pub_;
  std::thread spinner_thread;
  void spinThread(void);
  float grid_spacing = 0.1;
  double max_accel_time;

};

typedef std::shared_ptr<IRRTStarAvoid> IRRTStarAvoidPtr;

}
}
