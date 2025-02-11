#pragma once
/*
Copyright (c) 2021, Marco Faroni CNR-STIIMA marco.faroni@stiima.cnr.it
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

#include <ros/ros.h>
#include <graph_core/multi_goal_selection/policies/policy_mab.h>

namespace multi_goal_selection
{

class PolicyMABExample: public PolicyMAB
{
public:
  PolicyMABExample(const std::string& name, const int& n_goals) : PolicyMAB(name, n_goals){}

  /* Dummy MAB that select arm with probability weighted by reward^2 of each arm */
  int selectNextArm()
  {
    ROS_ERROR_ONCE("ok");
    int iarm = std::uniform_int_distribution<int>(0,n_goals_-1)(gen_);
    double sum = 0.0;
    for (int idx=0;idx<n_goals_;idx++)
      sum += expected_reward_[idx]*expected_reward_[idx];

    double rnd = std::uniform_real_distribution<double>(0.0,sum)(gen_);
    for(int idx=0; idx<n_goals_; idx++)
    {
      if(rnd < expected_reward_[idx]*expected_reward_[idx])
        return idx;
      rnd -= expected_reward_[idx]*expected_reward_[idx];
    }
    ROS_ERROR("should not be here");
    return iarm;
  }

  /* Dummy MAB that updates the reward as average of past rewards*/
  void updateState(const int& i_goal, const double& reward)
  {   
    ROS_ERROR_ONCE("ok");
    //expected_reward_[i_goal] *=  pull_counter_[i_goal];
    //expected_reward_[i_goal] +=  reward;
    pull_counter_[i_goal]++;
    //expected_reward_[i_goal] /=  pull_counter_[i_goal];
  }

  std::string toString()
  {
    std::string str = "example";
    return str;
  }

};
typedef std::shared_ptr<PolicyMABExample> PolicyMABExamplePtr;

}
