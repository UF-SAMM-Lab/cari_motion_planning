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

#include <irrt_star_avoid/solvers/time_avoid_rrt_star.h>

namespace pathplan
{

bool TimeAvoidRRTStar::addStartTree(const TreePtr &start_tree, const double &max_time)
{
  assert(start_tree);
  start_tree_ = start_tree;
  start_tree->time_avoid_ = true;
  start_tree->use_net = use_net;
  ROS_INFO_STREAM("tree using STAP net:"<<use_net);
  return setProblem(max_time);
}

void TimeAvoidRRTStar::setNumChildChecks(int nc) {
  start_tree_->num_child_checks = nc;
}

bool TimeAvoidRRTStar::addGoal(const NodePtr &goal_node, const double &max_time)
{
  solved_ = false;
  goal_node_ = goal_node;
  ROS_INFO("adding the goal, setting the problem");
  start_tree_->goal_node_ = goal_node_;
  return setProblem(max_time);
}

void TimeAvoidRRTStar::addNode(const NodePtr &new_node) {
  start_tree_->addNode(new_node);
}

bool TimeAvoidRRTStar::setProblem(const double &max_time)
{
  init_ = false;
  if (!start_tree_)
    return false;
  if (!goal_node_)
    return false;
  goal_cost_ = 0.0;

  best_utopia_ = std::numeric_limits<double>::max()/utopia_tolerance_/1.05;
  init_ = true;
  NodePtr new_node;
  // ROS_INFO_STREAM("attempting connection between start and goal node:"<<goal_node_);
  // if(start_tree_->connectToNode(goal_node_, new_node,max_time))  //for direct connection to goal
  // {
  double node_time=0;
  std::vector<Eigen::Vector3f> avoid_ints;
  float last_pass_time;
  float min_human_dist;
  double cost_start_to_goal = metrics_->cost(start_tree_->getNodes()[0], goal_node_, node_time,avoid_ints,last_pass_time,min_human_dist);

  if (checker_->checkPath(start_tree_->getNodes()[0]->getConfiguration(), goal_node_->getConfiguration()) && (cost_start_to_goal<std::numeric_limits<double>::infinity()))
  {
    start_tree_->addNode(goal_node_);

    ConnectionPtr conn = std::make_shared<Connection>(start_tree_->getNodes()[0], goal_node_);

    conn->setParentTime(node_time);
    conn->setAvoidIntervals(avoid_ints,last_pass_time,min_human_dist);
    conn->setMinTime(start_tree_->inv_max_speed_,start_tree_->min_accel_time);
    
    conn->add();      
    conn->setCost(cost_start_to_goal);

  //   std::cout<<"nodes 1:\n";
  //   for (NodePtr n:start_tree_->getNodes()) std::cout<<*n;
    // ROS_INFO_STREAM("new node"<<new_node);
    // goal_node_ = new_node;
    // start_tree_->goal_node_ = goal_node_;
    // start_tree_->directSolution(goal_node_);
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    // ROS_INFO_STREAM("created the solution");
    solution_->setTree(start_tree_);
    // ROS_INFO_STREAM("created the start tree");

    path_cost_ = goal_node_->parent_connections_[0]->getCost();
    // ROS_INFO_STREAM("path cost is "<<path_cost_);
    sampler_->setCost(path_cost_);
    // ROS_INFO_STREAM("set sampler cost");
    // ROS_INFO_STREAM("added goal node "<<goal_node_);

    // std::cout<<"nodes 2:\n";
    // for (NodePtr n:start_tree_->getNodes()) std::cout<<*n;
    solved_ = true;
    // PATH_COMMENT_STREAM("A direct solution is found\n" << *solution_);
  }
  else
  {
    // std::cout<<"adding node:"<<*goal_node_;
    start_tree_->addNode(goal_node_);
    path_cost_ = std::numeric_limits<double>::max();
    // PATH_COMMENT_STREAM("No direct solution is found " << path_cost_);
  }
  cost_=path_cost_+goal_cost_;
  // std::vector<NodePtr> all_nodes = start_tree_->getNodes();
  // std::cout<<"all nodes:\n";
  // for (int i=0;i<all_nodes.size();i++) {
  //   std::cout<<all_nodes[i]<<std::endl;
  //   std::cout<<*all_nodes[i]<<std::endl;
  // }
  return true;
}

bool TimeAvoidRRTStar::config(const ros::NodeHandle& nh)
{
  RRT::config(nh);
  solved_ = false;
  if (!nh.getParam("rewire_radius",r_rewire_))
  {
    ROS_DEBUG("%s/rewire_radius is not set. using 2.0*max_distance",nh.getNamespace().c_str());
    r_rewire_=2.0*max_distance_;
  }
  if (!nh.getParam("samples_per_batch",samples_per_batch))
  {
    ROS_DEBUG("%s/samples_per_batch is not set. using 5",nh.getNamespace().c_str());
    samples_per_batch=5;
  }
  return true;
}
//samples config space, then updates solution
bool TimeAvoidRRTStar::update(PathPtr& solution)
{
  return update(sampler_->sample(), solution);
}

//samples config space, then updates solution
bool TimeAvoidRRTStar::updateBatch(PathPtr& solution)
{
  std::vector<Eigen::VectorXd> samples;
  for (int i=0;i<samples_per_batch;i++) samples.push_back(sampler_->sample());
  return update(samples, solution);
}

//this is for adding new nodes
bool TimeAvoidRRTStar::update(const Eigen::VectorXd& configuration, PathPtr& solution)
{
  // std::cout<<"sample "<<configuration.transpose()<<std::endl;
  // PATH_COMMENT_STREAM("init:"<<init_);
  if (!init_) {
    // PATH_COMMENT_STREAM("exit update, not init");
    return false;
  }
  // PATH_COMMENT_STREAM("cost:"<<cost_<<",utopia_tolerance_:"<<utopia_tolerance_<<",best_utopia_:"<<best_utopia_);
  // if (cost_ <= utopia_tolerance_ * best_utopia_)
  // {
  //   ROS_INFO("Already optimal (121)");
  //   std::cout<<cost_<<std::endl;
  //   solution=solution_;
  //   completed_=true;
  //   return true;
  // }

  double old_path_cost;
  // PATH_COMMENT_STREAM("getting solution cost");
  //for time-avoidance, cost is time to reach goal node
  if (solution_) {
    old_path_cost = start_tree_->costToNode(goal_node_);
  } else {
    old_path_cost = std::numeric_limits<double>::max();
  }

  // PATH_COMMENT_STREAM("old path cost:");

  // PATH_COMMENT_STREAM("number of nodes in tree0: "<<start_tree_->getNumberOfNodes());
  bool improved = start_tree_->rewire(configuration, r_rewire_);
  // PATH_COMMENT_STREAM("number of nodes in tree: "<<start_tree_->getNumberOfNodes());
  if (improved)
  {
    // PATH_COMMENT_STREAM("improved"<<goal_node_->parent_connections_.size()<<", "<<start_tree_->costToNode(goal_node_)<<", "<<old_path_cost);
    //this is a problem
    // PATH_COMMENT_STREAM("solution_improved");
    // std::vector<ConnectionPtr> goal_connections = start_tree_->getConnectionToNode(goal_node_);
    // for (int i=0;i<goal_connections.size();i++) {
    //   double n_time = 0;
    //   std::vector<Eigen::Vector3f> avoid_ints;
    //   float min_human_dist;
    //   float last_pass_time;
    //   double cost = metrics_->cost(goal_connections[i]->getParent(),goal_connections[i]->getChild(), n_time, avoid_ints, last_pass_time, min_human_dist);
    //   ROS_INFO_STREAM("node:"<<goal_connections[i]->getChild()->getConfiguration().transpose()<<", cost:"<<cost<<", p min t:"<<goal_connections[i]->getParent()->min_time<<", n_time:"<<n_time<<","<<min_human_dist<<","<<last_pass_time);
    //   goal_connections[i]->setParentTime(n_time);
    //   goal_connections[i]->setCost(cost);
    // }
    if (start_tree_->costToNode(goal_node_)  >= (old_path_cost - 1e-8))
      return false;
    // PATH_COMMENT_STREAM(start_tree_->costToNode(goal_node_));
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    // PATH_COMMENT_STREAM("solution:\n"<<*solution_);
    solution_->setTree(start_tree_);

    path_cost_ = start_tree_->costToNode(goal_node_) ;
    cost_ = path_cost_;
    sampler_->setCost(cost_);
  }
  solution = solution_;
  // PATH_COMMENT_STREAM("num nodes "<<start_tree_->getNodes().size());
  return improved;
}

//this is for adding new nodes
bool TimeAvoidRRTStar::update(const std::vector<Eigen::VectorXd>& configurations, PathPtr& solution)
{
  // std::cout<<"sample "<<configuration.transpose()<<std::endl;
  // PATH_COMMENT_STREAM("init:"<<init_);
  if (!init_) {
    // PATH_COMMENT_STREAM("exit update, not init");
    return false;
  }


  double old_path_cost;
  // PATH_COMMENT_STREAM("getting solution cost");
  //for time-avoidance, cost is time to reach goal node
  if (solution_) {
    old_path_cost = start_tree_->costToNode(goal_node_);
  } else {
    old_path_cost = std::numeric_limits<double>::max();
  }

  // PATH_COMMENT_STREAM("old path cost:");

  // PATH_COMMENT_STREAM("number of nodes in tree0: "<<start_tree_->getNumberOfNodes());
  bool improved = start_tree_->rewireBatch(configurations, r_rewire_);
  // PATH_COMMENT_STREAM("number of nodes in tree: "<<start_tree_->getNumberOfNodes());
  if (improved)
  {
    // std::cout<<"getting cost to goal1\n";
    double tmp_cost = start_tree_->costToNode(goal_node_);
    // std::cout<<"done getting cost to goal1\n";
    if (tmp_cost >= (old_path_cost - 1e-8)) 
      return false;
    // PATH_COMMENT_STREAM(start_tree_->costToNode(goal_node_));
    // std::cout<<"making solution\n";
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    // std::cout<<"done making solution\n";
    // PATH_COMMENT_STREAM("solution:\n"<<*solution_);
    solution_->setTree(start_tree_);
    // std::cout<<"getting cost to goal2\n";
    path_cost_ = start_tree_->costToNode(goal_node_) ;
    // std::cout<<"done getting cost to goal2\n";
    cost_ = path_cost_;
    sampler_->setCost(cost_);
  }
  solution = solution_;
  // PATH_COMMENT_STREAM("num nodes "<<start_tree_->getNodes().size());
  return improved;
}

//I assume this if the function for rewiring old news to the new parent
bool TimeAvoidRRTStar::update(const NodePtr& n, PathPtr& solution)
{
  if (!init_)
    // ROS_INFO("exit update, init");
    return false;
  // ROS_INFO("old node cfg:");
  // std::cout<<n->getConfiguration()<<std::endl;
  if (cost_ <= utopia_tolerance_ * best_utopia_)
  {
    completed_=true;
    solution=solution_;
    return true;
  }
  //current path min cost is the cost (time of occupancy) of the goal node
  double old_path_cost = goal_node_->parent_connections_.at(0)->getCost();
  //double r_rewire = std::min(start_tree_->getMaximumDistance(), r_rewire_factor_ * sampler_->getSpecificVolume() * std::pow(std::log(start_tree_->getNumberOfNodes())/start_tree_->getNumberOfNodes(),1./dof_));
  double r_rewire = start_tree_->getMaximumDistance();
  bool improved = start_tree_->rewireToNode(n, r_rewire);

  if (improved)
  {
    if (goal_node_->parent_connections_.at(0)->getCost() >= (old_path_cost - 1e-8))
      return false;
    // ROS_INFO_STREAM("here");
    solution_ = std::make_shared<Path>(start_tree_->getConnectionToNode(goal_node_), metrics_, checker_);
    solution_->setTree(start_tree_);
    //path min cost is the cost (time of occupancy) of the goal node
    path_cost_ = goal_node_->parent_connections_.at(0)->getCost();
    cost_ = path_cost_;
    sampler_->setCost(cost_);
  }
  solution = solution_;
  return improved;
}

bool TimeAvoidRRTStar::solve(PathPtr &solution, const unsigned int& max_iter, const double &max_time)
{
  ros::WallTime tic = ros::WallTime::now();
  bool improved = false;
  for (unsigned int iter = 0; iter < max_iter; iter++)
  {
    if (update(solution))
    {
      ROS_DEBUG("Improved in %u iterations", iter);
      solved_ = true;
      improved = true;
    }
    if((ros::WallTime::now()-tic).toSec()>=0.98*max_time)
      break;
  }
  return improved;
}

TreeSolverPtr TimeAvoidRRTStar::clone(const MetricsPtr& metrics, const CollisionCheckerPtr& checker, const SamplerPtr& sampler)
{
  TimeAvoidRRTStarPtr new_solver = std::make_shared<TimeAvoidRRTStar>(metrics,checker,sampler);
  new_solver->config(nh_);
  return new_solver;
}

}
