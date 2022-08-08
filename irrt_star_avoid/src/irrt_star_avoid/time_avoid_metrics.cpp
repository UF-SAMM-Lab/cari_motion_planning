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

#include <irrt_star_avoid/time_avoid_metrics.h>


namespace pathplan
{

TimeAvoidMetrics::TimeAvoidMetrics(const Eigen::VectorXd& max_speed, const Eigen::VectorXd& max_acc, const double& nu, const double& t_pad, bool use_iso15066):
  Metrics(),
  max_speed_(max_speed),
  max_acc_(max_acc),
  nu_(nu),
  t_pad_(t_pad),
  use_iso15066_(use_iso15066)
{
  Eigen::VectorXd dt = max_speed_.array()/max_acc_.array();
  max_dt = 0.8*dt.maxCoeff();
  PATH_COMMENT_STREAM("max dt:"<<max_dt);
  int i = 0;
  for (i=0;i<dt.size();i++) if (dt[i]==max_dt) break;
  slow_joint = i;
  inv_max_speed_=max_speed_.cwiseInverse();

  name="time avoid metrics";

}

bool TimeAvoidMetrics::interval_intersection(float avd_int_1_start, float avd_int_1_end, float conn_int_start, float conn_int_end) {
  if ((avd_int_1_start-t_pad_<conn_int_start) && (conn_int_start<avd_int_1_end+t_pad_)) return true;
  if ((avd_int_1_start-t_pad_<conn_int_end) && (conn_int_end<avd_int_1_end+t_pad_)) return true;
  if ((conn_int_start<avd_int_1_start+t_pad_) && (avd_int_1_end-t_pad_<conn_int_end)) return true;
  return false;
}

//JF - need node1 instead of config1
double TimeAvoidMetrics::cost(const NodePtr& parent,
                              const NodePtr& new_node, double &near_time, std::vector<Eigen::Vector3f> &avoid_ints, float &last_pass_time, float &min_human_dist)
{

    // PATH_COMMENT_STREAM("time avoid metrics cost fn 1");

    //determine min time to complete straight line path from parent to config
    Eigen::VectorXd dist_new = new_node->getConfiguration()-parent->getConfiguration();
    
    // PATH_COMMENT_STREAM("dist_new:"<<dist_new);



    // double p1 = parent->getConfiguration()[slow_joint]+0.5*max_acc_[slow_joint]*max_dt*max_dt;
    // double p2 = new_node->getConfiguration()[slow_joint]+0.5*max_acc_[slow_joint]*max_dt*max_dt;
    // double dt3 = (p2-p1)/max_speed_[slow_joint];
    // double tf = 2*max_dt+dt3;

    double time_new = (inv_max_speed_.cwiseProduct(dist_new)).cwiseAbs().maxCoeff()+2*max_dt;
    // PATH_COMMENT_STREAM("time_new:"<<time_new<<","<<tf);
    double node_time_new = time_new;
    //get cost of parent
    //requires extended node data
    double c_near = 0.0;
    // if (parent.min_time == std::numeric_limits<double>::infinity()) {
    //     c_near = parent->parent_connections_.at(0)->getCost(); //connections store cost to node at end of connection
    // } else {
    //     c_near = parent.min_time;
    // }
    if (!parent->parent_connections_.empty()) {
      c_near = parent->min_time;//parent_connections_.at(0)->getParent()->min_time+parent->parent_connections_.at(0)->getCost(); 
    }
    near_time = c_near;
    // get min cost to get to new node from near parent
    double c_new = c_near + time_new;

    bool success = true; //bool to set false if path can not be found
    
    //convert robot to point cloud, or
    //determine which model points the robot is in collision with at configuration
    //probably need to write a function to descritize robot bounding boxes or cylinders to point clouds
    //call the parallel checker to speed things up
    avoid_ints.clear();
    last_pass_time = std::numeric_limits<float>::infinity();
    // PATH_COMMENT_STREAM("last pass time:"<<last_pass_time);
    // PATH_COMMENT_STREAM("last pass time:"<<parent->getConfiguration());
    // PATH_COMMENT_STREAM("last pass time:"<<new_node->getConfiguration());

    //if connection already exists, then get avoid ints and last pass time from connection, else call pc avoid checker
    bool conn_found = false;
    for (const ConnectionPtr& conn : parent->child_connections_){
      if (conn->getChild()==new_node) {
        avoid_ints = conn->getAvoidIntervals();
        last_pass_time = conn->getLPT();
        // min_human_dist = conn->getMinHumanDist();
        conn_found = true;
        break;
      }
    }
    if (!conn_found) {
      pc_avoid_checker->checkPath(parent->getConfiguration(), new_node->getConfiguration(), avoid_ints, last_pass_time);

    }
    // PATH_COMMENT_STREAM("last pass time:"<<last_pass_time);
    if (c_new>last_pass_time){
        success = false;
        // break;
    }
    //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
    if (!avoid_ints.empty() && success){
        bool found_avoid = false;
        double tmp_time = 0.0;
        double tmp_c_new = c_new;
        for (int r=0;r<avoid_ints.size();r++) {
            // std::cout<<avoid_ints[r].transpose()<<std::endl;
            //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
            //repeat check with the time to reach the parent node when parent time is increased
            //ensuring the entire path from parent to new node is not in an avoidance interval
            //need to check if connection intveral is within connection interval
            //if any part of the avoidance interval overlaps the connection interval (near_time to tmp_c_new)
            // std::cout<<parent->getConfiguration().transpose()<<"->"<<new_node->getConfiguration().transpose()<<std::endl;
            // std::cout<<avoid_ints[r][0]<<","<<avoid_ints[r][1]<<","<<tmp_c_new<<","<<near_time<<std::endl;
            if (interval_intersection(avoid_ints[r][0],avoid_ints[r][1],near_time,tmp_c_new)) {
            // if (((avoid_ints[r][0]-t_pad_<tmp_c_new) && (tmp_c_new<avoid_ints[r][1]+t_pad_)) || ((avoid_ints[r][0]-t_pad_<near_time) && (near_time<avoid_ints[r][1]+t_pad_))) {
                tmp_time = avoid_ints[r][1]+node_time_new + t_pad_;
                if (tmp_time>tmp_c_new) {
                    tmp_c_new = tmp_time;
                    near_time = avoid_ints[r][1]+t_pad_;
                }
                if (use_iso15066_) { //in case a little more slow down prevents an iso15066 slow down
                  double extra_time_to_avoid_slow_down = 0.0;
                  double prev_slow_cost = std::numeric_limits<double>::infinity();
                  for (int i=0;i<100;i++) {
                    double slow_cost = pc_avoid_checker->checkISO15066(parent->getConfiguration(),new_node->getConfiguration(),dist_new.norm(),near_time,tmp_time+i*0.1,1);
                    std::cout<<"iso15066 cost:"<<slow_cost<<std::endl;
                    if ((i>0) && (slow_cost>prev_slow_cost)) {
                      extra_time_to_avoid_slow_down = prev_slow_cost-tmp_time;
                      break;
                    }
                    prev_slow_cost = slow_cost;
                  }
                  tmp_time += extra_time_to_avoid_slow_down;
                  std::cout<<" new iso time:"<<tmp_time<<std::endl;
                  if (tmp_time>tmp_c_new) {
                      tmp_c_new = tmp_time;
                      // near_time = avoid_ints[r][1]+t_pad_+extra_time_to_avoid_slow_down;
                  }
                }
                found_avoid = true;
            } else if (found_avoid) {
                c_new = tmp_c_new;
                found_avoid = false;
                break;
            }
        }
        if (found_avoid) {
            c_new = tmp_c_new;
        }
        c_new = std::max(c_new,tmp_c_new);

        // std::cout<<"c_new, near time"<<c_new<<","<<near_time<<std::endl;
      } else if (use_iso15066_) {
        std::cout<<"c_new changed from "<<c_new<<" to ";
        c_new = pc_avoid_checker->checkISO15066(parent->getConfiguration(),new_node->getConfiguration(),dist_new.norm(),c_near,c_new,dist_new.norm()/0.1);
        std::cout<<c_new<<" with iso15066\n";
      }
    // PATH_COMMENT_STREAM("cnear:"<<c_near<<", c new:"<<c_near + time_new<<", c_new:"<<c_new<<", parent min time:"<<parent->min_time<<", last pass time:"<<last_pass_time<<","<<avoid_ints.size());

    //return inf cost if not success
    if (!success) {
        c_new = std::numeric_limits<double>::infinity();
    }

    //<to-do> must also return the near_time to assign to parent_time_occupancy

    return c_new;
}

double TimeAvoidMetrics::utopia(const Eigen::VectorXd &configuration1, const Eigen::VectorXd &configuration2)
{
  return (inv_max_speed_.cwiseProduct(configuration1 - configuration2)).cwiseAbs().maxCoeff()+nu_*(configuration2-configuration1).norm();
}


}
