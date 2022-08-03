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
  max_accels_.resize(m_dof);
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
      max_accels_(idx)=bounds.max_acceleration_;
    }
  }


  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  COMMENT("create metrics");
  metrics=std::make_shared<pathplan::TimeAvoidMetrics>(max_velocity_,max_accels_,1e-2, t_pad_);
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
  
  std::vector<double> ws_lb_param;

  if (m_nh.getParam("workspace_lower_bounds_xyz",ws_lb_param))
  {
    for (int i=0;i<3;i++) workspace_lb[i] = ws_lb_param[i];
  } else {
    ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={-1,-1,0.5}");
  }  

  std::vector<double> ws_ub_param;

  if (m_nh.getParam("workspace_upper_bounds_xyz",ws_ub_param))
  {
    for (int i=0;i<3;i++) workspace_ub[i] = ws_ub_param[i];
  } else {
    ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={1,1,2.5}");
  }

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
  solver->setInvMaxTime(max_velocity_.cwiseInverse(),metrics->max_dt);

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
  std::vector<Eigen::VectorXd> wp_sequence;
  std::vector<double> wp_times;
  time_parameterize(waypoints,waypoint_times,wp_sequence,wp_times);

  robot_trajectory::RobotTrajectoryPtr trj(new robot_trajectory::RobotTrajectory(robot_model_,group_));

  COMMENT("processing %zu aypoints..", waypoints.size());
  double prev_wpt_time = 0;
  for (int i=0;i<wp_sequence.size();i++) {
    Eigen::VectorXd waypoint = wp_sequence[i];
    COMMENT("processing waypoint");
    moveit::core::RobotState wp_state=start_state;
    wp_state.setJointGroupPositions(group_,waypoint);
    wp_state.update();
    trj->addSuffixWayPoint(wp_state,wp_times[i]-prev_wpt_time);
    prev_wpt_time = wp_times[i];
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

void IRRTStarAvoid::computeTransitions(double &tf, double &v0, double &vf, double &a1, double &a2, double v_max, double &t1, double &t2, bool recur) {
    ROS_INFO_STREAM("start tf:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1:"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
  if (vf<=v0) a2 = -abs(a2);
  double a = 0.5*(a1-a2);
  double b = a2*tf-(vf-v0)*(a2/a1);
  double c = 1+0.5*(vf-v0)*(vf-v0)/a1-vf*tf;
  ROS_INFO_STREAM("a:"<<a<<",b:"<<b<<",c:"<<c);
  double dt2_1;
  double dt2_2;
  double dt2;
  if (a!=0) {
    double r=b*b-4*a*c;
    ROS_INFO_STREAM("r:"<<r);
    if (r<0) {
      double a_tf = a2*a2;
      double b_tf = 4*a*vf-2*a2*a2/a1*(vf-v0);
      double c_tf = -(4*a*(1+0.5*(vf-v0)*(vf-v0)/a1)-(vf-v0)*(vf-v0)*(a2/a1)*(a2/a1));
      double tf1 = (-b_tf+sqrt(b_tf*b_tf-4*a_tf*c_tf))/(2*a_tf);
      double tf2= (-b_tf-sqrt(b_tf*b_tf-4*a_tf*c_tf))/(2*a_tf);
      tf = std::max(tf1,tf2)+0.01;
      b = a2*tf-(vf-v0)*(a2/a1);
      c = 1+0.5*(vf-v0)*(vf-v0)/a1-vf*tf;
      r = b*b-4*a*c;
    }
    dt2_1 = (-b+sqrt(r))/(2*a);
    dt2_2 = (-b-sqrt(r))/(2*a);
    if (dt2_1>0) {
        if (dt2_2>0){
            dt2 = std::min(dt2_1,dt2_2);
        } else {
            dt2 = dt2_1;
        }
    } else if (dt2_2>0) {
        dt2=dt2_2;
    } else {
        ROS_INFO_STREAM("something went wrong");
        t1 = 0;
        t2 = tf;
    }
    // ROS_INFO_STREAM("dt2 solution: "<<dt2))

  } else {
    dt2 = -c/b;
  }
  t1 = (vf-v0)/a1-(a2/a1)*dt2;
  t2 = tf-dt2;
  if (!recur) {
    return;
  }
  ROS_INFO_STREAM("t1:"<<t1<<", t2:"<<t2);
  if (t1>tf) {
    a2 = -a2;
    computeTransitions(tf,v0,vf,a1,a2,v_max,t1,t2,false);
    ROS_INFO_STREAM("tfc:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
  }
  if (t1<0) {
    a1 = -a1;
    computeTransitions(tf,v0,vf,a1,a2,v_max,t1,t2,false);
    ROS_INFO_STREAM("tfb:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
  }
  if (dt2<0) {
    double tf_2, t1_2, t2_2, a1_2, a2_2;
    tf_2 = tf;
    t1_2 = t1;
    t2_2 = t2;
    a1_2 = a1;
    a2_2 = -a2;

    computeTransitions(tf_2,v0,vf,a1_2,a2_2,v_max,t1_2,t2_2,false);
    ROS_INFO_STREAM("tf2:"<<tf_2<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1_2<<",a2:"<<a2_2<<",t1:"<<t1_2<<",t2:"<<t2_2);
    tf = (1+0.5*(vf-v0)*(vf-v0)/a1)/vf;
    ROS_INFO_STREAM("tf2:"<<tf);
    if (tf_2<tf) {
      tf=tf_2;
      t1 = t1_2;
      t2 = t2_2;
      a1 = a1_2;
      a2 = a2_2;
    } else {
        computeTransitions(tf,v0,vf,a1,a2,v_max,t1,t2,false);
        ROS_INFO_STREAM("tfa:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
    }
  }
  double v_mid = v0+a1*t1;
    ROS_INFO_STREAM("v_mid:"<<v_mid<<", v_max:"<<v_max);
  if (v_mid>v_max) {
    t1 = (v_max-v0)/a1;
    dt2 = (vf-v_max)/a2;
    double p1 = v0*t1+0.5*a1*t1*t1;
    double p2 = 1-vf*dt2+0.5*a2*dt2*dt2;
    double dt3 = (p2-p1)/v_max;
    tf = t1+dt2+dt3;
    t2 = tf-dt2;
  }
}

void IRRTStarAvoid::time_parameterize(std::vector<Eigen::VectorXd> waypoints, std::vector<double> waypoint_times, std::vector<Eigen::VectorXd> &slow_sequence, std::vector<double> &slow_seq_times) {
    std::vector<Eigen::VectorXd> q_array;
    std::vector<int> q_splits;
    int n_dof = int(max_velocity_.size());
    Eigen::VectorXd dq(n_dof);
    Eigen::VectorXd phase_space_to_q_dot(n_dof);
    std::vector<double> phase_space_slopes;
    int hz = 1000;
    int num_q_steps_per_hz = 2000;
    Eigen::VectorXd max_d_q_dot_allowed_per_step = (1.0/double(hz))*num_q_steps_per_hz*max_accels_;
    std::vector<double> segment_acc_lim;
    std::vector<int> wp_t_steps;
    for (int i=1;i<int(waypoints.size());i++) {
        Eigen::VectorXd diff = waypoints[i]-waypoints[i-1];
        int num_pts = ceil((diff.array().abs()/max_velocity_.array()).maxCoeff()*num_q_steps_per_hz*hz+1);
        int t_steps = ceil((waypoint_times[i]-waypoint_times[i-1])*hz);
        wp_t_steps.push_back(t_steps);
        Eigen::VectorXd dq = diff/num_pts;

        // ROS_INFO_STREAM("i:"<<i<<", dq:"<<dq.transpose());
        std::vector<Eigen::VectorXd> q_pts;
        for (int j=1;j<num_pts+1;j++) q_pts.push_back(diff*j/num_pts+waypoints[i-1]);
        Eigen::VectorXd old_q_dot = phase_space_to_q_dot;
        phase_space_to_q_dot = dq*hz;
        Eigen::VectorXd d_q_dot = phase_space_to_q_dot-old_q_dot;
        Eigen::ArrayXd num_t_steps_for_q_dot_change = d_q_dot.array().abs()*max_d_q_dot_allowed_per_step.array().inverse();
        phase_space_slopes.push_back(1/ceil(num_t_steps_for_q_dot_change.maxCoeff())/num_pts);
        double time_steps_for_q_acceleration = (dq.array().abs()/max_accels_.array()).maxCoeff()*double(hz*num_q_steps_per_hz);
        // ROS_INFO_STREAM("i:"<<i<<", steps for acc:"<<time_steps_for_q_acceleration);
        segment_acc_lim.push_back(1.0/time_steps_for_q_acceleration/(num_pts-1));
        q_splits.push_back(int(q_array.size()));
        q_array.insert(q_array.end(),q_pts.begin(),q_pts.end());
    }
    phase_space_slopes[0] = 0;
    phase_space_slopes.push_back(0);
    q_splits.push_back(int(q_array.size()));
    // ROS_INFO_STREAM("q_array:"<<q_array.size());
    std::vector<double> x_axis;
    std::vector<Eigen::VectorXd> sequence;
    std::vector<double> seq_times;
    std::vector<int> seq_ids;
    for (int j=1;j<int(waypoints.size());j++) {
        double tf =  wp_t_steps[j-1];
        double v0 = phase_space_slopes[j-1];
        double vf = phase_space_slopes[j];
        double a1 = segment_acc_lim[j-1];
        double a2 = segment_acc_lim[j-1];
        double t1;
        double t2;
        computeTransitions(tf,v0,vf,a1,a2,double(num_q_steps_per_hz)/double((q_splits[j]-q_splits[j-1])), t1, t2, true);
        ROS_INFO_STREAM("tf:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
        if (tf>0) {
          double intercept = -0.5*a1*t1*t1;
          double slope = v0+a1*t1;
          // ROS_INFO_STREAM("j:"<<j<<", slope:"<<slope);
          double p = 0;
          // ROS_INFO_STREAM("j:"<<j<<", q_splits:"<<q_splits.size());
          int num_pts = q_splits[j]-q_splits[j-1];
          // ROS_INFO_STREAM("j:"<<j<<", num_pts:"<<num_pts);
          for (int i = 0;i<int(ceil(tf));i++) {
              double i_dbl = double(i);
              if (i_dbl<t1) {
                  // ROS_INFO_STREAM("j:"<<j<<", here:"<<i_dbl);
                  // ROS_INFO_STREAM("tf:"<<tf<<",v0:"<<v0<<",vf:"<<vf<<",a1"<<a1<<",a2:"<<a2<<",t1:"<<t1<<",t2:"<<t2);
                  p=v0*i_dbl+0.5*a1*i_dbl*i_dbl;
                  // ROS_INFO_STREAM("j:"<<j<<", p:"<<p);
              } else if (i_dbl>=t2) {
                  // ROS_INFO_STREAM("j:"<<j<<", here:2");
                  p=1-vf*(tf-i_dbl)+0.5*a2*(tf-i_dbl)*(tf-i_dbl);
              } else {
                  // ROS_INFO_STREAM("j:"<<j<<", here:3");
                  p=slope*i_dbl+intercept;
              }

              // ROS_INFO_STREAM("j:"<<j<<", p:"<<p);
              // std::cout<<p<<",";
              int q_id = int(p*num_pts)+q_splits[j-1];
              // std::cout<<q_id;
              sequence.push_back(q_array.at(q_id));
              // std::cout<<",";
              seq_ids.push_back(q_id);
              seq_times.push_back(i_dbl/double(hz));
          }
        } else {
            sequence.push_back(q_array.at(0));
            seq_ids.push_back(0);
            seq_times.push_back(0.0);
        }
    }
    int sample_hz = 100;
    for (int j=0;j<sequence.size()-int(hz/sample_hz);j+=int(hz/sample_hz)) {
      slow_sequence.push_back(sequence[j]);
      slow_seq_times.push_back(double(j)/double(hz));
    }
    slow_sequence.push_back(sequence.back());
    slow_seq_times.push_back(double(sequence.size())/double(hz));

}

}
}
