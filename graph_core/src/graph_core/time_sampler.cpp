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


#include <graph_core/time_sampler.h>

namespace pathplan
{
TimeBasedInformedSampler::TimeBasedInformedSampler(const Eigen::VectorXd& start_configuration,
                                                   const Eigen::VectorXd& stop_configuration,
                                                   const Eigen::VectorXd& lower_bound,
                                                   const Eigen::VectorXd& upper_bound,
                                                   const Eigen::VectorXd& max_speed,
                                                   const double& cost):
  InformedSampler(start_configuration,stop_configuration,lower_bound,upper_bound,cost),
  max_speed_(max_speed)
{
  inv_max_speed_=max_speed_.cwiseInverse();
  utopia_=(stop_configuration_-start_configuration_).cwiseProduct(inv_max_speed_).cwiseAbs().maxCoeff();
  setCost(cost); // it is necessary because InformedSampler could change the cost if less than focii distance
  ROS_DEBUG_STREAM("max_speed" << max_speed.transpose());
  ROS_DEBUG_STREAM("inv_max_speed_" << inv_max_speed_.transpose());
  ROS_DEBUG_STREAM("utopia_" << utopia_);

}

void TimeBasedInformedSampler::setCost(const double &cost)
{
  InformedSampler::setCost(cost*std::sqrt(ndof_)*max_speed_.maxCoeff());
  cost_ = cost;
  inf_cost_ = cost_ >= std::numeric_limits<double>::infinity();

  if (cost_ < utopia_)
  {
    ROS_WARN("cost is %f, utopia_ is %f", cost_, utopia_);
    cost_ = utopia_;
  }


  specific_volume_=1.0;
  for (unsigned int idx=0;idx<ndof_;idx++)
  {
    if (cost_==inf_cost_)
      specific_volume_*=(upper_bound_(idx)-lower_bound_(idx));
    else
      specific_volume_*=cost_*max_speed_(idx);
  }
}


bool TimeBasedInformedSampler::inBounds(const Eigen::VectorXd& q)
{
  for (unsigned int iax = 0; iax < ndof_; iax++)
  {
    if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
    {
      return false;
    }
  }
  if (inf_cost_)
    return true;

  double check_cost=((q-start_configuration_).cwiseProduct(inv_max_speed_)).cwiseAbs().maxCoeff()+
      ((q-stop_configuration_ ).cwiseProduct(inv_max_speed_)).cwiseAbs().maxCoeff();
  // ROS_INFO_STREAM("check cost:"<<check_cost);
  return  (check_cost<cost_);

}



Eigen::VectorXd TimeBasedInformedSampler::sample()
{

  if (cost_>=std::numeric_limits<double>::infinity())
    return InformedSampler::sample();

  Eigen::VectorXd q(ndof_);

  unsigned int iteration=0;
  while (true)
  {

    if (iteration++>100000)
    {
      ROS_ERROR_THROTTLE(1,"Unable to find a sample in the informed set, cost =%f",cost_);
      return InformedSampler::sample();
    }
    Eigen::VectorXd tmp = 0.5*cost_*max_speed_;
    // ROS_INFO_STREAM("upper_bound_:"<<upper_bound_.transpose());
    Eigen::VectorXd upper_lim = tmp.cwiseMin(upper_bound_-ellipse_center_);
    // ROS_INFO_STREAM("upper_lim:"<<upper_lim.transpose());
    tmp = -1.0*tmp;
    Eigen::VectorXd lower_lim = tmp.cwiseMax(lower_bound_-ellipse_center_);
    // ROS_INFO_STREAM("lower_lim:"<<lower_lim.transpose());
    Eigen::VectorXd rand_part = (0.5*Eigen::MatrixXd::Random(ndof_, 1)+Eigen::MatrixXd::Constant(ndof_,1,0.5));
    // ROS_INFO_STREAM("rnd:"<<rand_part.transpose());
    Eigen::VectorXd q2 = rand_part.cwiseProduct(upper_lim-lower_lim)+lower_lim;
    // ROS_INFO_STREAM("tmp:"<<q2.transpose());
    // Eigen::VectorXd rng = (upper_bound_-ellipse_center_).cwiseMin(ellipse_center_-lower_bound_);
    // for (int i = 0;i<ndof_;i++) tmp[i] = fmod(tmp[i],rng[i]);
    // ROS_INFO_STREAM("tmp:"<<tmp.transpose());
    q=ellipse_center_+q2;
    // ROS_INFO_STREAM("ellipse_center_:"<<ellipse_center_.transpose());
    // ROS_INFO_STREAM("q:"<<q.transpose());
    if (inBounds(q))
    {
      return q;
    }

  }

}


}  // namespace pathplan
