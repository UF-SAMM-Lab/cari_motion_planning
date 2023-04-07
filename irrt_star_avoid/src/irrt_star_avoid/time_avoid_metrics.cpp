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

    double time_new = (inv_max_speed_.cwiseProduct(dist_new)).cwiseAbs().maxCoeff()+0*2*max_dt;
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
    min_human_dist = 1.0;
    if (use_iso15066_) {
      // std::cout<<"c_new changed from "<<c_new<<" to ";
      c_new = pc_avoid_checker->checkISO15066(parent->getConfiguration(),new_node->getConfiguration(),dist_new.norm(),c_near,c_new,ceil(dist_new.norm()/0.1),min_human_dist);
      // std::cout<<c_new<<" with iso15066\n";
    }
    //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
    if (!avoid_ints.empty() && success){
        bool found_avoid = false;
        double tmp_time = 0.0;
        double tmp_c_new = c_new;
        double extra_time_to_avoid_slow_down;
        double prev_slow_cost;
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
                  extra_time_to_avoid_slow_down = std::numeric_limits<double>::infinity();
                  prev_slow_cost = std::numeric_limits<double>::infinity();
                  for (int i=0;i<5;i++) {
                    double slow_cost = pc_avoid_checker->checkISO15066(parent->getConfiguration(),new_node->getConfiguration(),dist_new.norm(),near_time,tmp_time+double(i),ceil(dist_new.norm()/0.1),min_human_dist);
                    // std::cout<<"iso15066 cost:"<<slow_cost<<std::endl;
                    // if ((i>0) && (slow_cost>prev_slow_cost)) {
                    //   extra_time_to_avoid_slow_down = prev_slow_cost-tmp_time;
                    // }
                    extra_time_to_avoid_slow_down = std::min(slow_cost-tmp_time,extra_time_to_avoid_slow_down);
                    
                    prev_slow_cost = slow_cost;
                  }
                  tmp_time += extra_time_to_avoid_slow_down;
                  // std::cout<<" new iso time:"<<tmp_time<<std::endl;
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
        // else if (use_iso15066_) { //in case a little more slow down prevents an iso15066 slow down
        //   extra_time_to_avoid_slow_down = 0.0;
        //   prev_slow_cost = std::numeric_limits<double>::infinity();
        //   for (int i=0;i<5;i++) {
        //     double slow_cost = pc_avoid_checker->checkISO15066(parent->getConfiguration(),new_node->getConfiguration(),dist_new.norm(),near_time,tmp_c_new+double(i),ceil(dist_new.norm()/0.1),min_human_dist);
        //     // std::cout<<"iso15066 cost:"<<slow_cost<<std::endl;
        //     // if ((i>0) && (slow_cost>prev_slow_cost)) {
        //     //   extra_time_to_avoid_slow_down = prev_slow_cost-tmp_time;
        //     // }
        //     extra_time_to_avoid_slow_down = std::min(slow_cost-tmp_c_new,extra_time_to_avoid_slow_down);
            
        //     prev_slow_cost = slow_cost;
        //   }
        //   tmp_c_new += extra_time_to_avoid_slow_down;
        //   c_new = tmp_c_new;
        // }
        
        c_new = std::max(c_new,tmp_c_new);

        // std::cout<<"c_new, near time"<<c_new<<","<<near_time<<std::endl;
      }
    // PATH_COMMENT_STREAM("cnear:"<<c_near<<", c new:"<<c_near + time_new<<", c_new:"<<c_new<<", parent min time:"<<parent->min_time<<", last pass time:"<<last_pass_time<<","<<avoid_ints.size());

    //return inf cost if not success
    if (!success) {
        c_new = std::numeric_limits<double>::infinity();
    }

    //<to-do> must also return the near_time to assign to parent_time_occupancy

    return c_new;
}

double TimeAvoidMetrics::cost(const Eigen::VectorXd& parent,
                              const Eigen::VectorXd& new_node, double &near_time)
{

    // PATH_COMMENT_STREAM("time avoid metrics cost fn 1");

    //determine min time to complete straight line path from parent to config
    Eigen::VectorXd dist_new = new_node-parent;
    
    // PATH_COMMENT_STREAM("dist_new:"<<dist_new);



    // double p1 = parent[slow_joint]+0.5*max_acc_[slow_joint]*max_dt*max_dt;
    // double p2 = new_node[slow_joint]+0.5*max_acc_[slow_joint]*max_dt*max_dt;
    // double dt3 = (p2-p1)/max_speed_[slow_joint];
    // double tf = 2*max_dt+dt3;

    double time_new = (inv_max_speed_.cwiseProduct(dist_new)).cwiseAbs().maxCoeff()+0*2*max_dt;
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
    c_near = near_time;//parent_connections_.at(0)->getParent()->min_time+parent->parent_connections_.at(0)->getCost(); 
    // near_time = c_near;
    ROS_INFO_STREAM(near_time);
    // get min cost to get to new node from near parent
    double c_new = c_near + time_new;

    bool success = true; //bool to set false if path can not be found
    
    //convert robot to point cloud, or
    //determine which model points the robot is in collision with at configuration
    //probably need to write a function to descritize robot bounding boxes or cylinders to point clouds
    //call the parallel checker to speed things up
    std::vector<Eigen::Vector3f> avoid_ints;
    float last_pass_time;
    // PATH_COMMENT_STREAM("last pass time:"<<last_pass_time);
    // PATH_COMMENT_STREAM("last pass time:"<<parent);
    // PATH_COMMENT_STREAM("last pass time:"<<new_node);

    //if connection already exists, then get avoid ints and last pass time from connection, else call pc avoid checker
    pc_avoid_checker->checkPath(parent, new_node, avoid_ints, last_pass_time);

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
        double extra_time_to_avoid_slow_down;
        double prev_slow_cost;
        for (int r=0;r<avoid_ints.size();r++) {
            // std::cout<<avoid_ints[r].transpose()<<std::endl;
            //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
            //repeat check with the time to reach the parent node when parent time is increased
            //ensuring the entire path from parent to new node is not in an avoidance interval
            //need to check if connection intveral is within connection interval
            //if any part of the avoidance interval overlaps the connection interval (near_time to tmp_c_new)
            // std::cout<<parent.transpose()<<"->"<<new_node.transpose()<<std::endl;
            // std::cout<<avoid_ints[r][0]<<","<<avoid_ints[r][1]<<","<<tmp_c_new<<","<<near_time<<std::endl;
            if (interval_intersection(avoid_ints[r][0],avoid_ints[r][1],near_time,tmp_c_new)) {
            // if (((avoid_ints[r][0]-t_pad_<tmp_c_new) && (tmp_c_new<avoid_ints[r][1]+t_pad_)) || ((avoid_ints[r][0]-t_pad_<near_time) && (near_time<avoid_ints[r][1]+t_pad_))) {
                tmp_time = avoid_ints[r][1]+node_time_new + t_pad_;
                if (tmp_time>tmp_c_new) {
                    tmp_c_new = tmp_time;
                    near_time = avoid_ints[r][1]+t_pad_;
                    ROS_INFO_STREAM(near_time);
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
        // else if (use_iso15066_) { //in case a little more slow down prevents an iso15066 slow down
        //   extra_time_to_avoid_slow_down = 0.0;
        //   prev_slow_cost = std::numeric_limits<double>::infinity();
        //   for (int i=0;i<5;i++) {
        //     double slow_cost = pc_avoid_checker->checkISO15066(parent,new_node,dist_new.norm(),near_time,tmp_c_new+double(i),ceil(dist_new.norm()/0.1),min_human_dist);
        //     // std::cout<<"iso15066 cost:"<<slow_cost<<std::endl;
        //     // if ((i>0) && (slow_cost>prev_slow_cost)) {
        //     //   extra_time_to_avoid_slow_down = prev_slow_cost-tmp_time;
        //     // }
        //     extra_time_to_avoid_slow_down = std::min(slow_cost-tmp_c_new,extra_time_to_avoid_slow_down);
            
        //     prev_slow_cost = slow_cost;
        //   }
        //   tmp_c_new += extra_time_to_avoid_slow_down;
        //   c_new = tmp_c_new;
        // }
        
        c_new = std::max(c_new,tmp_c_new);

        // std::cout<<"c_new, near time"<<c_new<<","<<near_time<<std::endl;
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

//JF - need node1 instead of config1
void TimeAvoidMetrics::cost(std::vector<std::tuple<const NodePtr,const NodePtr,double,std::vector<Eigen::Vector3f>,float,float,double>>& node_datas)
{
  if (node_datas.size()==0) return;

  std::vector<std::tuple<Eigen::VectorXd,Eigen::VectorXd,std::vector<Eigen::Vector3f>,float>> configurations;
  configurations.reserve(node_datas.size());
  Eigen::MatrixXd parents(node_datas.size(),std::get<0>(node_datas[0])->getConfiguration().size());
  Eigen::MatrixXd children(node_datas.size(),std::get<0>(node_datas[0])->getConfiguration().size());

  for (int i=0;i<node_datas.size();i++) {
    std::tuple<Eigen::VectorXd,Eigen::VectorXd,std::vector<Eigen::Vector3f>,float> config;
    std::get<0>(config) = std::get<0>(node_datas[i])->getConfiguration();
    parents.row(i) = std::get<0>(node_datas[i])->getConfiguration();
    std::get<1>(config) = std::get<1>(node_datas[i])->getConfiguration();
    children.row(i) = std::get<1>(node_datas[i])->getConfiguration();
    // std::cout<<i<<", "<<std::get<0>(node_datas[i])->getConfiguration().transpose()<<", "<<std::get<1>(node_datas[i])->getConfiguration().transpose()<<std::endl;
    configurations.push_back(config);
  }

  pc_avoid_checker->checkMutliplePaths(configurations);
  Eigen::MatrixXd dist_new = parents-children;
  Eigen::MatrixXd tmp_mat = dist_new.array().abs().rowwise()*inv_max_speed_.transpose().array();
  Eigen::VectorXd time_new = tmp_mat.rowwise().maxCoeff();
  for (int i=0;i<node_datas.size();i++) {
    // std::cout<<i<<", "<<std::get<0>(node_datas[i])->getConfiguration().transpose()<<", "<<std::get<1>(node_datas[i])->getConfiguration().transpose()<<std::endl;
      //determine min time to complete straight line path from parent to config
    // const Eigen::VectorXd parent_cfg = std::get<0>(node_datas[i])->getConfiguration();
    // const Eigen::VectorXd child_cfg = std::get<1>(node_datas[i])->getConfiguration();
    // Eigen::VectorXd dist_new = parent_cfg-child_cfg;

    // double time_new = (inv_max_speed_.cwiseProduct(dist_new)).maxCoeff()+0*2*max_dt;
    double node_time_new = time_new[i];
    //get cost of parent
    //requires extended node data
    double c_near = 0.0;
    if (!std::get<0>(node_datas[i])->parent_connections_.empty()) {
      c_near = std::get<0>(node_datas[i])->min_time;//parent_connections_.at(0)->getParent()->min_time+parent->parent_connections_.at(0)->getCost(); 
    }
    std::get<2>(node_datas[i]) = c_near;    
    // get min cost to get to new node from near parent
    std::get<6>(node_datas[i]) = c_near + node_time_new;
    // std::cout<<"conn time:"<<std::get<6>(node_datas[i])<<std::endl;
    bool success = true; //bool to set false if path can not be found
    std::get<4>(node_datas[i]) = std::get<3>(configurations[i]);
    std::get<3>(node_datas[i]) = std::get<2>(configurations[i]);
    // std::cout<<"intervals:";
    // for (int a=0;a<std::get<3>(node_datas[i]).size();a++) std::cout<<std::get<3>(node_datas[i])[a].transpose()<<";";
    // std::cout<<std::endl;
    if (std::get<6>(node_datas[i])>std::get<4>(node_datas[i])){
        success = false;
        // break;
    }
    std::get<5>(node_datas[i]) = 1.0;
    if (use_iso15066_) {
      std::get<6>(node_datas[i]) = pc_avoid_checker->checkISO15066(parents.row(i),children.row(i),dist_new.row(i).norm(),c_near,std::get<6>(node_datas[i]),ceil(dist_new.row(i).norm()/0.1),std::get<5>(node_datas[i]));
    }
    //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
    if (!std::get<3>(node_datas[i]).empty() && success){
        bool found_avoid = false;
        double tmp_time = 0.0;
        double tmp_c_new = std::get<6>(node_datas[i]);
        double extra_time_to_avoid_slow_down;
        double prev_slow_cost;
        for (int r=0;r<std::get<3>(node_datas[i]).size();r++) {
            //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
            //repeat check with the time to reach the parent node when parent time is increased
            //ensuring the entire path from parent to new node is not in an avoidance interval
            //need to check if connection intveral is within connection interval
            //if any part of the avoidance interval overlaps the connection interval (near_time to tmp_c_new)
            if (interval_intersection(std::get<3>(node_datas[i])[r][0],std::get<3>(node_datas[i])[r][1],std::get<2>(node_datas[i]),tmp_c_new)) {
                tmp_time = std::get<3>(node_datas[i])[r][1]+node_time_new + t_pad_;
                if (tmp_time>tmp_c_new) {
                    tmp_c_new = tmp_time;
                    std::get<2>(node_datas[i]) = std::get<3>(node_datas[i])[r][1]+t_pad_;
                }
                if (use_iso15066_) { //in case a little more slow down prevents an iso15066 slow down
                  extra_time_to_avoid_slow_down = std::numeric_limits<double>::infinity();
                  prev_slow_cost = std::numeric_limits<double>::infinity();
                  for (int i=0;i<5;i++) {
                    double slow_cost = pc_avoid_checker->checkISO15066(parents.row(i),children.row(i),dist_new.row(i).norm(),std::get<2>(node_datas[i]),tmp_time+double(i),ceil(dist_new.row(i).norm()/0.1),std::get<5>(node_datas[i]));
                    extra_time_to_avoid_slow_down = std::min(slow_cost-tmp_time,extra_time_to_avoid_slow_down);
                    
                    prev_slow_cost = slow_cost;
                  }
                  tmp_time += extra_time_to_avoid_slow_down;
                  if (tmp_time>tmp_c_new) {
                      tmp_c_new = tmp_time;
                  }
                }
                found_avoid = true;
            } else if (found_avoid) {
                std::get<6>(node_datas[i]) = tmp_c_new;
                found_avoid = false;
                break;
            }
        }
        if (found_avoid) {
            std::get<6>(node_datas[i]) = tmp_c_new;
        } 
        
        std::get<6>(node_datas[i]) = std::max(std::get<6>(node_datas[i]),tmp_c_new);
      }

    //return inf cost if not success
    if (!success) {
        std::get<6>(node_datas[i]) = std::numeric_limits<double>::infinity();
    }
    // std::cout<<"conn time end:"<<std::get<6>(node_datas[i])<<std::endl;
  }

  // return;

  //   // ROS_INFO_STREAM("num configs:"<<configurations.size());
  // // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  // // JF- this would be the place for the neural network
  // int num_cfg = node_datas.size();
  // if (num_cfg<1) return;
  // int max_cfg_per_batch = pc_avoid_checker->getMaxCfgPerBatch();
  // int num_batches = std::ceil((double)num_cfg/(double)max_cfg_per_batch);
  // int num_cfg_per_batch = std::ceil((double)num_cfg/(double)num_batches);
  // ROS_INFO_STREAM("num_batches:"<<num_batches);
  // ROS_INFO_STREAM("num_cfg_per_batch:"<<num_cfg_per_batch);
  // std::vector<std::tuple<Eigen::VectorXd,Eigen::VectorXd,std::vector<Eigen::Vector3f>,float>> configurations;
  // configurations.resize(num_cfg_per_batch);
  // for (int k=0;k<num_batches;k++) {
  //   // ROS_INFO_STREAM("batch:"<<i);
  //   int num_cfg_per_this_batch = std::min(num_cfg_per_batch,num_cfg-k*num_cfg_per_batch);
  //   configurations.clear();
  //   for (int i=k*num_cfg_per_batch;i<k*num_cfg_per_batch+num_cfg_per_this_batch;i++) {
  //     std::tuple<Eigen::VectorXd,Eigen::VectorXd,std::vector<Eigen::Vector3f>,float> config;
  //     std::get<0>(config) = std::get<0>(node_datas[i])->getConfiguration();
  //     std::get<1>(config) = std::get<1>(node_datas[i])->getConfiguration();
  //     configurations.push_back(config);
  //   }
  //   pc_avoid_checker->checkBatchPaths(configurations);
  //   for (int i=k*num_cfg_per_batch;i<k*num_cfg_per_batch+num_cfg_per_this_batch;i++) {
  //       //determine min time to complete straight line path from parent to config
  //     Eigen::VectorXd dist_new = std::get<1>(node_datas[i])->getConfiguration()-std::get<0>(node_datas[i])->getConfiguration();

  //     double time_new = (inv_max_speed_.cwiseProduct(dist_new)).cwiseAbs().maxCoeff()+0*2*max_dt;
  //     double node_time_new = time_new;
  //     //get cost of parent
  //     //requires extended node data
  //     double c_near = 0.0;
  //     if (!std::get<0>(node_datas[i])->parent_connections_.empty()) {
  //       c_near = std::get<0>(node_datas[i])->min_time;//parent_connections_.at(0)->getParent()->min_time+parent->parent_connections_.at(0)->getCost(); 
  //     }
  //     std::get<2>(node_datas[i]) = c_near;    
  //     // get min cost to get to new node from near parent
  //     std::get<6>(node_datas[i]) = c_near + time_new;
  //     bool success = true; //bool to set false if path can not be found
  //     std::get<4>(node_datas[i]) = std::get<3>(configurations[i]);
  //     std::get<3>(node_datas[i]) = std::get<2>(configurations[i]);
  //     if (std::get<6>(node_datas[i])>std::get<4>(node_datas[i])){
  //         success = false;
  //         // break;
  //     }
  //     std::get<5>(node_datas[i]) = 1.0;
  //     if (use_iso15066_) {
  //       std::get<6>(node_datas[i]) = pc_avoid_checker->checkISO15066(std::get<0>(node_datas[i])->getConfiguration(),std::get<1>(node_datas[i])->getConfiguration(),dist_new.norm(),c_near,std::get<6>(node_datas[i]),ceil(dist_new.norm()/0.1),std::get<5>(node_datas[i]));
  //     }
  //     //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
  //     if (!std::get<3>(node_datas[i]).empty() && success){
  //         bool found_avoid = false;
  //         double tmp_time = 0.0;
  //         double tmp_c_new = std::get<6>(node_datas[i]);
  //         double extra_time_to_avoid_slow_down;
  //         double prev_slow_cost;
  //         for (int r=0;r<std::get<3>(node_datas[i]).size();r++) {
  //             //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
  //             //repeat check with the time to reach the parent node when parent time is increased
  //             //ensuring the entire path from parent to new node is not in an avoidance interval
  //             //need to check if connection intveral is within connection interval
  //             //if any part of the avoidance interval overlaps the connection interval (near_time to tmp_c_new)
  //             if (interval_intersection(std::get<3>(node_datas[i])[r][0],std::get<3>(node_datas[i])[r][1],std::get<2>(node_datas[i]),tmp_c_new)) {
  //                 tmp_time = std::get<3>(node_datas[i])[r][1]+node_time_new + t_pad_;
  //                 if (tmp_time>tmp_c_new) {
  //                     tmp_c_new = tmp_time;
  //                     std::get<2>(node_datas[i]) = std::get<3>(node_datas[i])[r][1]+t_pad_;
  //                 }
  //                 if (use_iso15066_) { //in case a little more slow down prevents an iso15066 slow down
  //                   extra_time_to_avoid_slow_down = std::numeric_limits<double>::infinity();
  //                   prev_slow_cost = std::numeric_limits<double>::infinity();
  //                   for (int i=0;i<5;i++) {
  //                     double slow_cost = pc_avoid_checker->checkISO15066(std::get<0>(node_datas[i])->getConfiguration(),std::get<1>(node_datas[i])->getConfiguration(),dist_new.norm(),std::get<2>(node_datas[i]),tmp_time+double(i),ceil(dist_new.norm()/0.1),std::get<5>(node_datas[i]));
  //                     extra_time_to_avoid_slow_down = std::min(slow_cost-tmp_time,extra_time_to_avoid_slow_down);
                      
  //                     prev_slow_cost = slow_cost;
  //                   }
  //                   tmp_time += extra_time_to_avoid_slow_down;
  //                   if (tmp_time>tmp_c_new) {
  //                       tmp_c_new = tmp_time;
  //                   }
  //                 }
  //                 found_avoid = true;
  //             } else if (found_avoid) {
  //                 std::get<6>(node_datas[i]) = tmp_c_new;
  //                 found_avoid = false;
  //                 break;
  //             }
  //         }
  //         if (found_avoid) {
  //             std::get<6>(node_datas[i]) = tmp_c_new;
  //         } 
          
  //         std::get<6>(node_datas[i]) = std::max(std::get<6>(node_datas[i]),tmp_c_new);
  //       }

  //     //return inf cost if not success
  //     if (!success) {
  //         std::get<6>(node_datas[i]) = std::numeric_limits<double>::infinity();
  //     }
  //   }
  // }
}


}
