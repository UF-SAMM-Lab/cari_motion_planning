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

#include <graph_core/util.h>
#include <graph_core/graph/node.h>
#include <parallel_robot_point_clouds/parallel_robot_point_clouds.h>
namespace pathplan
{
class Metrics;
typedef std::shared_ptr<Metrics> MetricsPtr;


// Euclidean metrics
class Metrics
{
protected:
  std::string name = "base metrics";
public:
  ParallelRobotPointCloudsPtr pc_avoid_checker;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Metrics();

  virtual double cost(const NodePtr& node1,
                      const NodePtr& node2);

  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);
  
  virtual double cost(const NodePtr& parent,
                              const NodePtr& new_node, double &near_time, std::vector<Eigen::Vector3f> &avoid_ints, float &last_pass_time, float &min_human_dist);

  virtual void cost(std::vector<std::tuple<const NodePtr,const NodePtr,double,std::vector<Eigen::Vector3f>,float,float,double>>& node_datas);

  virtual double utopia(const NodePtr& node1,
                      const NodePtr& node2);

  virtual double utopia(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2);

  virtual void setPointCloudChecker(ParallelRobotPointCloudsPtr pc_avoid_checker_) {
    pc_avoid_checker=pc_avoid_checker_;
  }

  std::string getName() {
    return name;
  }

  virtual MetricsPtr clone();

};

}
