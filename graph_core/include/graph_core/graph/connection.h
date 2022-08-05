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
namespace pathplan
{
class Connection : public std::enable_shared_from_this<Connection>
{
protected:
  NodePtr parent_;
  NodePtr child_;
  double cost_ = std::numeric_limits<double>::infinity(); //JF - init connection cost to infinity?
  bool added_ = false;
  double euclidean_norm_;
  double time_;
  double likelihood_;
private:
  double parent_time_ = 0;
  std::vector<Eigen::Vector3f> avoidance_intervals_;
  float last_pass_time_;
  double min_time_;
  float min_human_dist;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Connection(const NodePtr& parent, const NodePtr& child, const double& time=0.0);
  ConnectionPtr pointer()
  {
    return shared_from_this();
  }

  virtual void add();
  virtual void remove();
  void setMinTime(Eigen::VectorXd inv_max_speed_, double max_accel_time);

  virtual bool isNet()
  {
    return false;
  }

  void setParentTime(const double& parent_time);

  const double& getParentTime()
  {
    return parent_time_;
  }

  void setCost(const double& cost);
  
  void setAvoidIntervals(std::vector<Eigen::Vector3f> avoid_ints, float last_pass_time, double min_separation_dist)
  {
    last_pass_time_ = last_pass_time;
    avoidance_intervals_ = avoid_ints;
    min_human_dist = min_separation_dist;
  }
  void setMinTime(const double& min_time)
  {
    min_time_ = min_time;
  }
  std::vector<Eigen::Vector3f> getAvoidIntervals()
  {
    return avoidance_intervals_;
  }
  float getLPT()
  {
    return last_pass_time_;
  }
  const double& getCost()
  {
    return cost_;
  }
  double norm()
  {
    return euclidean_norm_;
  }
  double getMinTime()
  {
    return min_time_;
  }
  bool getAdded()
  {
    return added_;
  }
  double getMinHumanDist()
  {
    return min_human_dist;
  }
  const NodePtr& getParent() const
  {
    return parent_;
  }
  const NodePtr& getChild() const
  {
    return child_;
  }

  void setLikelihood(const double& likelihood){likelihood_=likelihood;}

  virtual ConnectionPtr clone();

  void flip();

  bool isParallel(const ConnectionPtr& conn, const double& toll = 1e-06);

  friend std::ostream& operator<<(std::ostream& os, const Connection& connection);
  ~Connection();
};



std::ostream& operator<<(std::ostream& os, const Connection& connection);

}
