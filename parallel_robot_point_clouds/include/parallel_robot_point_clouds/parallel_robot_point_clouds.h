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
// #include <graph_core/moveit_collision_checker.h>
#include <thread>
#include <mutex>
#include <avoidance_intervals/avoidance_model.h>
#include <chrono>
#include <ctime>
#include <ratio>
#include<iterator> // for iterators
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rosdyn_core/primitives.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

namespace pathplan
{
bool vectorEqual(Eigen::Vector3f vec1, Eigen::Vector3f vec2);

class ParallelRobotPointClouds
{
private:
  double start_time;
  Eigen::VectorXd start_cfg;
  Eigen::VectorXd inv_max_speed_;
  int num_confs_per_thread;
  std::vector<bool> model_pts_already_checked;
  std::vector<bool> cfg_is_avoid;
  ros::Publisher pub_pt;
  std::vector<Eigen::Vector3f> avoid_ints;
  std::vector<std::vector<Eigen::Vector3f>> th_avoid_ints;
  std::vector<std::vector<bool>> th_avoid_status;
  std::mutex mtx;
  std::vector<Eigen::MatrixXf> link_raw_pts;
  std::ofstream avoid_ints_file;
  std::vector<std::vector<std::string>> avoid_ints_output_thread;
protected:
  double t_pad_=0;
  Eigen::VectorXd max_q_dot_;
  bool fcl_check;
  ros::NodeHandle nh;
  int threads_num_;
  int thread_iter_=0;
  bool stop_check_;
  bool at_least_a_collision_;
  double grid_spacing_;
  double max_cart_acc_;
  double reaction_time;
  double dist_dec_;
  double term1_;
  double min_hrc_distance_;

  std::string group_name_;
  double min_distance_;

  std::vector<std::vector<std::pair<int,std::vector<double>>>> queues_;
  std::vector<std::thread> threads;
  std::vector<float> min_dists;
  std::vector<Eigen::VectorXd> tangential_vels;
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_;
  std::vector<Eigen::Vector3f> collision_points;
  std::vector<Eigen::Vector3f> link_bb_offsets;
  std::vector<Eigen::Vector3f> link_boxes;
  std::vector<const robot_state::LinkModel*> links;
  std::vector<const robot_state::LinkModel*> robot_links;

  int n_dof = 6;

  std::mutex stop_mutex;
  void resetQueue();
  void queueUp(const Eigen::VectorXd &q);
  void checkAllQueues(std::vector<Eigen::Vector3f> &combined_avoidance_intervals, float &last_pass_time);
  void collisionThread(int thread_idx);
  void queueConnection(const Eigen::VectorXd& configuration1,
                       const Eigen::VectorXd& configuration2);
  void sort_reduce_link_pts(std::vector<Eigen::Vector3f> &link_pts);
  void show_transformed_pts(Eigen::MatrixXf &transformed_pts);
  void pt_intersection(Eigen::Isometry3f &link_transform, int link_id, int thread_id);

  collision_detection::CollisionRequest req_;
  collision_detection::CollisionResult res_;
  std::vector<Eigen::MatrixXf> link_pts;
  std::vector<Eigen::Vector3f> link_pts_vec;
  int num_link_pts = 0;
  const moveit::core::JointModelGroup* joint_model_group;
  ssm15066::DeterministicSSMPtr ssm_;
  rosdyn::ChainPtr chain_;

public:
  avoidance_intervals::modelPtr model_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  ParallelRobotPointClouds(ros::NodeHandle node_handle, moveit::core::RobotModelConstPtr robot_model,
                                 const std::string& group_name,const avoidance_intervals::modelPtr& model,
                                  const Eigen::VectorXd& max_vels,
                                 const int& threads_num=4,
                                 const double& min_distance = 0.01,
                                 const double& grid_spacing = 0.05);

  ~ParallelRobotPointClouds();

  // virtual void setPlanningSceneMsg(const moveit_msgs::PlanningScene& msg);

  // virtual void setPlanningScene(planning_scene::PlanningScenePtr &planning_scene);

  virtual void checkPath(const Eigen::VectorXd& configuration1,
                                               const Eigen::VectorXd& configuration2, 
                                               std::vector<Eigen::Vector3f> &avoid_ints,
                                               float &last_pass_time);

  // virtual bool checkConnFromConf(const ConnectionPtr& conn,
  //                                const Eigen::VectorXd& this_conf);

  // virtual bool checkConnections(const std::vector<ConnectionPtr>& connections);

  // virtual CollisionCheckerPtr clone();

  // std::string getGroupName() override
  // {
  //   return group_name_;
  // }

  // virtual planning_scene::PlanningScenePtr getPlanningScene() override
  // {
  //   return planning_scene_;
  // }
  std::vector<ros::Publisher> vis_pubs;
  ros::Publisher pub_robot_pt;
  void displayRobot(int i,Eigen::Vector3f sides,Eigen::Vector3f pos, Eigen::Quaternionf quat);
  void displayCollisionPoint(float radius,Eigen::Vector3f pos);
  void clearRobot(void);
  double checkISO15066(const Eigen::VectorXd& configuration1,
                                              const Eigen::VectorXd& configuration2, double length, float t1, float t2, unsigned int nsteps, float &min_human_dist) ;
  void updatePlanningScene(const planning_scene::PlanningScenePtr &planning_scene);

};
typedef std::shared_ptr<ParallelRobotPointClouds> ParallelRobotPointCloudsPtr;
}  // namaspace pathplan