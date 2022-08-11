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


#include <irrt_star_avoid/time_informed_sampler.h>

namespace pathplan
{


Eigen::VectorXd TimeInformedSampler::sample()
{
  // ROS_INFO("sampling");
  if (inf_cost_)
  {
    Eigen::VectorXd rand_samp = 0.5*Eigen::MatrixXd::Random(ndof_, 1)+0.5*Eigen::MatrixXd::Ones(ndof_, 1);
    Eigen::VectorXd sample = l_box_+rand_samp.cwiseProduct(q_range_);
    // PATH_COMMENT_STREAM("sample:\n"<<sample);
    return sample;
  }
  else
  {
    Eigen::VectorXd ball(ndof_);
    for (int iter = 0; iter < 100; iter++)
    {
      ball.setRandom();
      ball *= std::pow(ud_(gen_), 1.0 / (double)ndof_) / ball.norm();

      Eigen::VectorXd q = q_range_.cwiseProduct(ball) + q_mid_;

      bool in_of_bounds = true;
      for (unsigned int iax = 0; iax < ndof_; iax++)
      {
        if (q(iax) > upper_bound_(iax) || q(iax) < lower_bound_(iax))
        {
          in_of_bounds = false;
          break;
        }
      }
      if (in_of_bounds)
        return q;
    }
    ROS_DEBUG_THROTTLE(0.1, "unable to find a feasible point in the hyperrectangle");
    Eigen::VectorXd sample = l_box_ + Eigen::MatrixXd::Random(ndof_, 1).cwiseProduct(q_range_);
    // PATH_COMMENT_STREAM("sample:\n"<<sample);
    return sample;
  }
}

bool TimeInformedSampler::inBounds(const Eigen::VectorXd& q)
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
  else
    return ((q - start_configuration_).norm() + (q - stop_configuration_).norm()) < cost_;

}

void TimeInformedSampler::setCost(const double &cost)
{
  cost_ = cost;
  inf_cost_ = cost_ >= std::numeric_limits<double>::infinity();

  // if (cost_ < utopia_)
  // {
  //   ROS_WARN("cost is %f, utopia_ is %f", cost_, utopia_);
  //   cost_ = utopia_;
  // }

  if (!inf_cost_)
  {
    //generate bounds of hyperrectangle
    q_dist_ = 0.5*(cost*max_vel_-(stop_configuration_-start_configuration_).cwiseAbs());
    l_box_ = start_configuration_.cwiseMin(stop_configuration_)-q_dist_;
    l_box_ = l_box_.cwiseMax(lower_bound_);
    u_box_ = start_configuration_.cwiseMax(stop_configuration_)+q_dist_;
    u_box_ = u_box_.cwiseMin(upper_bound_);
  }
  else {
    l_box_ = lower_bound_;
    u_box_ = upper_bound_;
  }

  q_range_ = (u_box_-l_box_);
  q_mid_ = 0.5*(u_box_+l_box_);


  specific_volume_=1.0;
  for (unsigned int idx=0;idx<ndof_;idx++)
  {
    if (cost_==inf_cost_)
      specific_volume_*=(upper_bound_(idx)-lower_bound_(idx));
    else
      specific_volume_*=cost_*max_vel_(idx);
  }
}


double TimeInformedSampler::getSpecificVolume()
{
  return specific_volume_;
}

}