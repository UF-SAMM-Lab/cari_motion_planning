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
#include <graph_core/moveit_collision_checker.h>
#include <thread>
#include <mutex>
#include <avoidance_intervals/avoidance_model.h>

namespace pathplan
{
class ParallelRobotPointClouds
{
protected:
  int threads_num_;
  int thread_iter_=0;
  bool stop_check_;
  bool at_least_a_collision_;
  avoidance_intervals::modelPtr model_;

  std::string group_name_;
  double min_distance_;

  std::vector<std::vector<std::vector<double>>> queues_;
  std::vector<std::thread> threads;
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_;
  std::vector<Eigen::Vector3f> collision_points;
  std::vector<Eigen::Vector3f> link_bb_offsets;
  std::vector<fcl::Boxf> link_boxes;
  std::vector<const robot_state::LinkModel*> links;

  int n_dof = 6;

  std::mutex stop_mutex;
  void resetQueue();
  void queueUp(const Eigen::VectorXd &q);
  void checkAllQueues(std::vector<Eigen::Vector3f> &combined_avoidance_intervals, float &last_pass_time);
  void collisionThread(int thread_idx);

  void queueConnection(const Eigen::VectorXd& configuration1,
                       const Eigen::VectorXd& configuration2);

  collision_detection::CollisionRequest req_;
  collision_detection::CollisionResult res_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  ParallelRobotPointClouds(const planning_scene::PlanningScenePtr& planning_scene,
                                 const std::string& group_name,const avoidance_intervals::modelPtr& model,
                                 const int& threads_num=4,
                                 const double& min_distance = 0.01);

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

};
typedef std::shared_ptr<ParallelRobotPointClouds> ParallelRobotPointCloudsPtr;
}  // namaspace pathplan
