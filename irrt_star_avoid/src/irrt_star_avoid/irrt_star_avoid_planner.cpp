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


#include <irrt_star_avoid/irrt_star_avoid_planner.h>




namespace pathplan {
namespace irrt_star_avoid {

IRRTStarAvoid::IRRTStarAvoid ( const std::string& name,
                       const std::string& group,
                       const moveit::core::RobotModelConstPtr& model ) :
  // m_nh(name),
  // spinner(1),
  PlanningContext ( name, group ),
  group_(group),
  m_gen(time(0))
{
  // spinner.start();
  m_nh=ros::NodeHandle(name);

  // vis_pub_ = m_nh.advertise<visualization_msgs::Marker>( "human_markers", 0 );
  
  // std::cout<<vis_pub_.getNumSubscribers()<<std::endl;
  // while (vis_pub_.getNumSubscribers()<1) {
  //   ros::Duration(1.0).sleep();
  //   std::cout<<vis_pub_.getNumSubscribers()<<std::endl;
  // }
  COMMENT("created vispub");
  // m_nh.setCallbackQueue(&m_queue);
  // spinner_thread = std::thread(&IRRTStarAvoid::spinThread,this);
  // spinner_thread.detach();
  COMMENT("create IRRTStarAvoid, name =%s, group = %s", name.c_str(),group.c_str());
  robot_model_=model;
  if (!robot_model_)
  {
    ROS_ERROR("robot model is not initialized");
  }
  COMMENT("model name = %s",robot_model_->getName().c_str());

  COMMENT("get joint model group");

  const moveit::core::JointModelGroup* jmg=robot_model_->getJointModelGroup(group);
  if (jmg==NULL)
    ROS_ERROR("unable to find JointModelGroup for group %s",group.c_str());


  COMMENT("get joint names of JointModelGroup=%s",jmg->getName().c_str());

  joint_names_=jmg->getActiveJointModelNames();
  m_dof=joint_names_.size();
  COMMENT("number of joints  = %u",m_dof);
  m_lb.resize(m_dof);
  m_ub.resize(m_dof);
  max_velocity_.resize(m_dof);
  t_pad_ = 0.0;

  COMMENT("read bounds");
  for (unsigned int idx=0;idx<m_dof;idx++)
  {
    COMMENT("joint %s",joint_names_.at(idx).c_str());
    const robot_model::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_.at(idx));
    if (bounds.position_bounded_)
    {
      m_lb(idx)=bounds.min_position_;
      m_ub(idx)=bounds.max_position_;
      max_velocity_(idx)=bounds.max_velocity_;
    }
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  COMMENT("create metrics");
  metrics=std::make_shared<pathplan::TimeAvoidMetrics>(max_velocity_,1e-2, t_pad_);
  base_metrics = std::static_pointer_cast<pathplan::Metrics>(metrics);

  // COMMENT("check planning scene");
  // if (!planning_scene_)
  //   ROS_ERROR("No planning scene available");

  if (!m_nh.getParam("display_bubbles",display_flag))
  {
    ROS_DEBUG("display_flag is not set, default=false");
    display_flag=false;
  }

  if (!m_nh.getParam("collision_distance",collision_distance))
  {
    ROS_DEBUG("collision_distance is not set, default=0.04");
    collision_distance=0.04;
  }

  if (!m_nh.getParam("collision_thread",collision_thread_))
  {
    ROS_DEBUG("collision_thread is not set, default=4");
    collision_thread_=4;
  }

  COMMENT("created IRRTStarAvoid");

  double refining_time=0;
  if (!m_nh.getParam("max_refine_time",refining_time))
  {
    ROS_DEBUG("refining_time is not set, default=30");
    refining_time=30;
  }
  m_max_refining_time=ros::WallDuration(refining_time);
  m_max_planning_time=ros::WallDuration(60);


  if (!m_nh.getParam("path_optimization",m_path_optimization))
  {
    ROS_DEBUG("refining_time is not set, default: false");
    m_path_optimization=false;
  }
  if (!m_nh.getParam("tube_sampler",m_tube_sampler))
  {
    ROS_DEBUG("refining_time is not set, default: false");
    m_tube_sampler=false;
  }

  if (!m_nh.getParam("max_iterations",max_iterations))
  {
    ROS_DEBUG("max_iterations is not set, default=false");
    max_iterations=0;
  }

  if (!m_nh.getParam("grid_spacing",grid_spacing))
  {
    ROS_DEBUG("grid_spacing is not set, default=false");
    grid_spacing=0.1;
  }

  if (!m_nh.getParam("allow_goal_collision",allow_goal_collision))
  {
    ROS_DEBUG("allow_goal_collision is not set, default=false");
    allow_goal_collision=false;
  }

  m_ud = std::uniform_real_distribution<double>(0, 1);
    
  COMMENT("init avoidance model");

  Eigen::Vector3f workspace_lb={-1,-1,0.5};
  Eigen::Vector3f workspace_ub={1,1,2.5};

  avoid_model_ = std::make_shared<avoidance_intervals::model>(workspace_lb,workspace_ub,grid_spacing,collision_thread_,m_nh);

}

void IRRTStarAvoid::setPlanningScene ( const planning_scene::PlanningSceneConstPtr& planning_scene )
{
  planning_scene_=planning_scene;
  COMMENT("create checker");
  planning_scene::PlanningScenePtr ps=planning_scene::PlanningScene::clone(planning_scene);
  checker=std::make_shared<pathplan::MoveitCollisionChecker>(ps,group_,collision_distance);

}

void IRRTStarAvoid::updateAvoidModel(std::vector<Eigen::VectorXf> avoid_pts) 
{
  avoid_model_->generate_model_cloud(avoid_pts);
}

// void IRRTStarAvoid::setPredictedPlanningScene ( const planning_scene::PlanningSceneConstPtr& planning_scene )
// {
//   predicted_planning_scene_=planning_scene;
//   COMMENT("create checker");
//   planning_scene::PlanningScenePtr ps=planning_scene::PlanningScene::clone(planning_scene);
//   predict_checker=std::make_shared<pathplan::MoveitCollisionChecker>(ps,group_,collision_distance);

// }

void IRRTStarAvoid::clear()
{

}


bool IRRTStarAvoid::solve ( planning_interface::MotionPlanDetailedResponse& res )
{



  if (display_flag)
  {
    if (!display)
      display=std::make_shared<pathplan::Display>(planning_scene_,group_);
    else
      display->clearMarkers();
  }

  ros::WallDuration max_planning_time=ros::WallDuration(request_.allowed_planning_time);
  ros::WallTime start_time = ros::WallTime::now();
  ros::WallTime refine_time = ros::WallTime::now();
  m_is_running=true;

  if (!planning_scene_)
  {
    ROS_ERROR("No planning scene available");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
    m_is_running=false;
    return false;
  }


  COMMENT("cloning the planning scene");
  planning_scene::PlanningScenePtr ptr=planning_scene::PlanningScene::clone(planning_scene_);
  COMMENT("init parallel collision checker");
  checker=std::make_shared<pathplan::ParallelMoveitCollisionChecker>(ptr,group_,collision_thread_,collision_distance);


  COMMENT("init parallel avoid point checker");
  point_cloud_checker=std::make_shared<pathplan::ParallelRobotPointClouds>(m_nh, ptr,group_,avoid_model_,collision_thread_,collision_distance,grid_spacing);

  COMMENT("settign metrics point cloud checker");
  metrics->setPointCloudChecker(point_cloud_checker);
  COMMENT("getting robot start state");
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);
  if (request_.start_state.joint_state.position.size()==0)
    start_state=planning_scene_->getCurrentState();
  else
    moveit::core::robotStateMsgToRobotState(request_.start_state,start_state);

  start_state.update();
  start_state.updateCollisionBodyTransforms();

  if (!start_state.satisfiesBounds())
  {
    ROS_FATAL("Start point is  Out of bound");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }
  Eigen::VectorXd start_conf;
  start_state.copyJointGroupPositions(group_,start_conf);

  COMMENT("checking start config");

  std::cout<<"start:\n"<<start_conf<<std::endl;

  if (!checker->check(start_conf))
  {
    ROS_ERROR("Start point is in collision");

    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;
    col_req.contacts = true;
    col_req.group_name=group_;
    planning_scene_->checkCollision(col_req,col_res,start_state);
    if (col_res.collision)
    {
      ROS_ERROR("Start state is colliding +++");
      for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
      {
        ROS_ERROR("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
      }
    }
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    m_is_running=false;
    return false;
  }


  pathplan::NodePtr start_node=std::make_shared<pathplan::Node>(start_conf);
  start_node->min_time = 0.0;
  COMMENT("creating a time informed sampler");
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::TimeInformedSampler>(m_lb, m_ub, m_lb, m_ub,max_velocity_);
  COMMENT("creating a time avoid rrt solver");
  std::shared_ptr<pathplan::TimeAvoidRRTStar> solver=std::make_shared<pathplan::TimeAvoidRRTStar>(metrics,checker,sampler);
  COMMENT("done created a time avoid rrt solver");
  solver->use_time_cost_ = true;
  PATH_COMMENT_STREAM("metrics name "<< solver->getMetricsName());
  // std::cin.ignore();
  if (!solver->config(m_nh))
  {
    ROS_ERROR("Unable to configure the planner");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    m_is_running=false;
    return false;
  }
  solver->addStart(start_node);
  solver->setInvMaxTime(max_velocity_.cwiseInverse());

  // m_queue.callAvailable();

  bool at_least_a_goal=false;

  COMMENT("checking goal configs");
  // joint goal
  for (unsigned int iGoal=0;iGoal<request_.goal_constraints.size();iGoal++)
  {
    ROS_DEBUG("Processing goal %u",iGoal);


    moveit_msgs::Constraints goal=request_.goal_constraints.at(iGoal);

    Eigen::VectorXd goal_configuration( goal.joint_constraints.size() );
    moveit::core::RobotState goal_state(robot_model_);

    for (auto c: goal.joint_constraints)
      goal_state.setJointPositions(c.joint_name,&c.position);
    goal_state.copyJointGroupPositions(group_,goal_configuration);

    std::cout<<"goal:\n"<<goal_configuration<<std::endl;
    goal_state.updateCollisionBodyTransforms();
    COMMENT("check collision on goal %u",iGoal);

    if (!checker->check(goal_configuration))
    {
      ROS_DEBUG("goal %u is in collision",iGoal);

      if (request_.goal_constraints.size()<5)
      {

        if (!goal_state.satisfiesBounds())
        {
          ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is  Out of bound");
        }

        collision_detection::CollisionRequest col_req;
        collision_detection::CollisionResult col_res;
        col_req.contacts = true;
        col_req.group_name=group_;
        planning_scene_->checkCollision(col_req,col_res,goal_state);
        if (col_res.collision)
        {
          ROS_INFO_STREAM("End state: " << goal_configuration.transpose()<<" is colliding");
          for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
          {
            ROS_INFO("contact between %s and %s",contact.first.first.c_str(),contact.first.second.c_str());
          }
        }
      }
      if (!allow_goal_collision) continue;
    }
    COMMENT("goal is valid");

    pathplan::NodePtr goal_node=std::make_shared<pathplan::Node>(goal_configuration);
    COMMENT("adding goal to tree");
    solver->addGoal(goal_node);
    at_least_a_goal=true;
  }


  //if there are no collision free goals, quit
  if (!at_least_a_goal)
  {
    ROS_ERROR("All goals are in collision");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
    m_is_running=false;
    return false;
  }


  // ROS_INFO_STREAM("push enter1");
  // std::cin.ignore();
  // metrics->pc_avoid_checker->model_->displayAllRequest();
  // ROS_INFO_STREAM("push enter2");
  // std::cin.ignore();
  //JF - should upate the model here.  Model takes time-varying predicted point cloud as input


  // ===============================
  // BEGINNING OF THE IMPORTANT PART
  // ===============================

  // searching initial solutions
  pathplan::PathPtr solution = solver->getSolution() ;
  bool found_a_solution=false;
  unsigned int iteration=0;
  ros::WallTime plot_time=ros::WallTime::now()-ros::WallDuration(100);
  double cost_of_first_solution;
  while (((ros::WallTime::now()-start_time)<max_planning_time) || (iteration<max_iterations))
  {
    PATH_COMMENT_STREAM("iteration number:"<<iteration);
    iteration++;
    if (m_stop)
    {
      ROS_INFO("Externally stopped");
      res.error_code_.val=moveit_msgs::MoveItErrorCodes::PREEMPTED;
      m_is_running=false;
      return false;
    }
    // PATH_COMMENT_STREAM("updating the solution");
    // solver->update(solution); //samples for a new node and udpates the node-graph
    if (solver->update(solution))
    {
      // ROS_DEBUG("Improved in %u iterations", iter);
      solver->setSolved(true);
      // improved = true;
    }
    // PATH_COMMENT_STREAM("found a solution:"<<found_a_solution<<", solver solved:"<<solver->solved());
    if (!found_a_solution && solver->solved())
    {
      assert(solution);
      ROS_INFO("Find a first solution (cost=%f) in %f seconds",solver->cost(),(ros::WallTime::now()-start_time).toSec());
      found_a_solution=true;
      cost_of_first_solution=solver->cost();
      ROS_INFO("path cost = %f",cost_of_first_solution);
      refine_time = ros::WallTime::now();
    }
    if (solver->completed())
    {
      ROS_INFO("Optimization completed (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }
    // PATH_COMMENT_STREAM("solver not complete");
    if (found_a_solution && ((ros::WallTime::now()-refine_time)>m_max_refining_time))
    {
      ROS_INFO("Refine time expired (cost=%f) in %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
      break;
    }

    // PATH_COMMENT_STREAM("solver still refining");
    if (found_a_solution)
    {
      ROS_DEBUG_THROTTLE(0.5,"solution improved from %f to %f",cost_of_first_solution,solution->cost());
      if (display_flag) {
        if ((ros::WallTime::now()-plot_time).toSec()>plot_interval_)
        {
          // PATH_COMMENT_STREAM("clearing markers");
          display->clearMarkers();
          // PATH_COMMENT_STREAM("displaying the tree");
          display->displayTree(solver->getStartTree());
          // PATH_COMMENT_STREAM("displaying the path");
          display->displayPath(solution,"pathplan",{0,1,0,1});
          // PATH_COMMENT_STREAM("done displaying");
          plot_time=ros::WallTime::now();
        }
      }
    }
  }

  // ROS_INFO_STREAM(*solver);

  if (!found_a_solution)
  {
    ROS_ERROR("unable to find a valid path");
    res.error_code_.val=moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    m_is_running=false;
    return false;
  }
  // ROS_INFO("solution improved from %f to %f",cost_of_first_solution,solution->cost());
  // ROS_INFO("path length = %f",solver->getSolution()->computeEuclideanNorm());

  if (!solver->completed())
  {
    ROS_INFO("Stopped (cost=%f) after %f seconds (%u iterations)",solver->cost(),(ros::WallTime::now()-start_time).toSec(),iteration);
  }

  // =========================
  // END OF THE IMPORTANT PART
  // =========================

  
  COMMENT("Get waypoints");
  std::vector<Eigen::VectorXd> waypoints=solution->getWaypoints();// PUT WAY POINTS
  COMMENT("Get waypoint timing");
  std::vector<double> waypoint_times=solution->getTiming();// PUT WAY POINTS
  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  COMMENT("processing %zu aypoints..", waypoints.size());
  double prev_wpt_time = 0;
  for (int i=0;i<waypoints.size();i++) {
    Eigen::VectorXd waypoint = waypoints[i];
    COMMENT("processing waypoint");
    moveit::core::RobotState wp_state=start_state;
    wp_state.setJointGroupPositions(group_,waypoint);
    wp_state.update();
    trj->addSuffixWayPoint(wp_state,waypoint_times[i]-prev_wpt_time);
    prev_wpt_time = waypoint_times[i];
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;


  res.processing_time_.push_back(wd.toSec());
  res.description_.emplace_back("plan");
  res.trajectory_.push_back(trj);

  res.error_code_.val=moveit_msgs::MoveItErrorCodes::SUCCESS;
  m_is_running=false;
  COMMENT("ok");

  // m_file.close();
  return true;
}

bool IRRTStarAvoid::solve ( planning_interface::MotionPlanResponse& res )
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);
  if(success)
  {
    res.trajectory_ = detailed_res.trajectory_.at(0);
  }
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = wd.toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool IRRTStarAvoid::terminate()
{
  m_stop=true;
  ros::Time t0=ros::Time::now();
  ros::Rate lp(50);
  while (ros::ok())
  {
    if (!m_is_running)
      return true;
    lp.sleep();
    if ((ros::Time::now()-t0).toSec()>5)
    {
      ROS_ERROR("Unable to stop planner %s of group %s",name_.c_str(),group_.c_str());
      return false;
    }
  }
}

void IRRTStarAvoid::spinThread(void) {
  while (ros::ok()) {
    m_queue.callAvailable(ros::WallDuration());
    ros::Duration(0.1).sleep();
  }
}

}
}
