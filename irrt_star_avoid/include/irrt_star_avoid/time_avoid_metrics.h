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

#include <graph_core/metrics.h>
#include <rosdyn_core/primitives.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <parallel_robot_point_clouds/parallel_robot_point_clouds.h>

namespace pathplan
{
class TimeAvoidMetrics;

typedef std::shared_ptr<TimeAvoidMetrics> TimeAvoidMetricsPtr;

// Avoidance metrics
class TimeAvoidMetrics: public Metrics
{
protected:
  Eigen::VectorXd max_speed_;
  Eigen::VectorXd max_acc_;
  double nu_=1e-2;
  double t_pad_=0;
  int slow_joint;
  bool use_iso15066_;

public:
  double max_dt;
  Eigen::VectorXd inv_max_speed_;
  // ParallelRobotPointCloudsPtr pc_avoid_checker;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimeAvoidMetrics(const Eigen::VectorXd& max_speed, const Eigen::VectorXd& max_acc, const double &nu=1e-2, const double &t_pad=0.0, bool use_iso15066=false);

  double cost(const NodePtr& parent,
                              const NodePtr& new_node, double &near_time, std::vector<Eigen::Vector3f> &avoid_ints, float &last_pass_time, float &min_human_dist);
  double cost(const Eigen::VectorXd& parent,
                              const Eigen::VectorXd& new_node,double &near_time);
                              
  double utopia(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);

  bool interval_intersection(float avd_int_1_start, float avd_int_1_end, float conn_int_start, float conn_int_end);


};

}