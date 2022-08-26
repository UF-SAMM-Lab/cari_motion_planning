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
#include <graph_core/sampler.h>
#include <eigen3/Eigen/Core>
#include <graph_core/util.h>
#include <ros/ros.h>
#include <random>

namespace pathplan
{

class TimeInformedSampler: public InformedSampler
{
protected:
  Eigen::VectorXd start_configuration_;
  Eigen::VectorXd stop_configuration_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  Eigen::VectorXd center_bound_;
  Eigen::VectorXd bound_width_;
  Eigen::VectorXd max_vel_;


  double cost_;
  unsigned int ndof_;


  Eigen::VectorXd q_dist_;
  Eigen::VectorXd l_box_;
  Eigen::VectorXd u_box_;
  Eigen::VectorXd q_mid_;
  Eigen::VectorXd q_range_;
  double utopia_;
  bool inf_cost_;
  double specific_volume_; // ndof-th root of volume of the hyperellipsoid divided by the volume of unit sphere

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> ud_;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TimeInformedSampler(const Eigen::VectorXd& start_configuration,
                  const Eigen::VectorXd& stop_configuration,
                  const Eigen::VectorXd& lower_bound,
                  const Eigen::VectorXd& upper_bound,
                  const Eigen::VectorXd& max_q_vel,
                  const double& cost = std::numeric_limits<double>::infinity()):
    InformedSampler(start_configuration,stop_configuration,lower_bound,upper_bound,cost),
    start_configuration_(start_configuration),
    stop_configuration_(stop_configuration),
    lower_bound_(lower_bound),
    upper_bound_(upper_bound),
    cost_(cost),
    max_vel_(max_q_vel),
    gen_{rd_()}//gen_(time(0))
  {

    std::cout<<"test3\n";
    Eigen::VectorXd inv_max_speed = max_q_vel.cwiseInverse();
    std::cout<<inv_max_speed<<std::endl;
    utopia_ = (stop_configuration-start_configuration).cwiseProduct(inv_max_speed).cwiseAbs().maxCoeff();

    std::cout<<"test4\n";
    ud_ = std::uniform_real_distribution<double>(0, 1);

    ndof_ = lower_bound_.rows();



    if (cost_ < std::numeric_limits<double>::infinity())
    {
      inf_cost_ = false;    
      //generate bounds of hyperrectangle
      q_dist_ = 0.5*(cost*max_q_vel-(stop_configuration-start_configuration).cwiseAbs());
      l_box_ = start_configuration.cwiseMin(stop_configuration)-q_dist_;
      l_box_ = l_box_.cwiseMax(lower_bound_);
      u_box_ = start_configuration.cwiseMax(stop_configuration)+q_dist_;
      u_box_ = u_box_.cwiseMin(upper_bound_);
      // setCost(cost);
    }
    else {
      inf_cost_ = true;
      l_box_ = lower_bound_;
      u_box_ = upper_bound_;
    }

    q_range_ = (u_box_-l_box_);
    q_mid_ = 0.5*(u_box_+l_box_);

    std::cout<<"box extents:\n" << q_dist_;
    std::cout<<"box mid:\n" << q_mid_;
    std::cout<<"box range:\n" << q_range_;
    std::cout<<"lower bounds" << l_box_;
    std::cout<<" upper bounds" << u_box_<<std::endl;
  }

  ros::Publisher sample_pub;
  virtual Eigen::VectorXd sample();
  void setCost(const double& cost);

  virtual bool inBounds(const Eigen::VectorXd& q);

  const double& getCost(){return cost_;}

  virtual double getSpecificVolume();

  virtual void sampleImproved(){}

  const Eigen::VectorXd& getLB(){return lower_bound_;}
  const Eigen::VectorXd& getUB(){return upper_bound_;}

  const Eigen::VectorXd& getStartConf(){return start_configuration_;}
  const Eigen::VectorXd& getStopConf(){return stop_configuration_;}

  const unsigned int& getDimension()const {return ndof_;}
};


typedef std::shared_ptr<TimeInformedSampler> TimeInformedSamplerPtr;

}  // namespace pathplan
