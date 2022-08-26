#include <parallel_robot_point_clouds/parallel_robot_point_clouds.h>

namespace pathplan
{
ParallelRobotPointClouds::ParallelRobotPointClouds(ros::NodeHandle node_handle,moveit::core::RobotModelConstPtr robot_model ,
                                                               const std::string& group_name, const avoidance_intervals::modelPtr& model,
                                                               const Eigen::VectorXd& max_vels,
                                                               const int& threads_num,
                                                               const double& min_distance,
                                                               const double& grid_spacing):
  nh(node_handle), group_name_(group_name), model_(model), max_q_dot_(max_vels), threads_num_(threads_num), min_distance_(min_distance), grid_spacing_(grid_spacing)
{

  ROS_INFO_STREAM("create parallel point clouds with "<<threads_num_<<" threads");
  // ROS_INFO_STREAM("press enter");
  // std::cin.ignore();

  if (threads_num<=0)
    throw std::invalid_argument("number of thread should be positive");

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  threads.resize(threads_num_);
  queues_.resize(threads_num_);
  th_avoid_ints.resize(threads_num_);
  // min_dists.resize(threads_num_);


  ROS_INFO_STREAM("parallel point clouds 1");
  req_.distance=false;
  req_.group_name=group_name;
  req_.verbose=false;
  req_.contacts=false;
  req_.cost=false;

  std::vector<std::vector<shapes::ShapeConstPtr>> link_shapes;

  // robot_state::RobotState robo_state(planning_scene->getCurrentState());
  // n_dof = robo_state.getRobotModel()->getVariableCount();
  // joint_model_group = robo_state.getJointModelGroup(group_name);
  links = robot_model->getLinkModels();
  link_shapes.resize(links.size());
  ROS_INFO_STREAM("num links"<<link_shapes.size());
  for (int i=0;i<int(links.size());i++) {
    std::string link_name = links[i]->getName();
    if (link_name.find("link")!=std::string::npos) {
      ROS_INFO_STREAM("link name "<<link_name);
    // if (link_name.substr(link_name.length()-4) == "link") {
      link_shapes[i] = links[i]->getShapes();
      if (link_shapes[i].size()>0){
        const Eigen::Vector3f& extents_tmp = links[i]->getShapeExtentsAtOrigin().cast<float>();
        Eigen::Vector3f extents = extents_tmp;
        const Eigen::Vector3f& offset_tmp = links[i]->getCenteredBoundingBoxOffset().cast<float>();
        Eigen::Vector3f offset = offset_tmp;

        ROS_INFO_STREAM("link name "<<link_name<<", extents:"<<extents.transpose()<<", offset:"<<offset.transpose());
        link_bb_offsets.push_back(offset);
        // Eigen::Isometry3d transform;
        // transform.setIdentity();
        // transform.translate(offset);
        // moveit::core::AABB aabb;
        // aabb.extendWithTransformedBox(transform,extents);
        // link_raw_pts.push_back(boxPts(extents));
        link_boxes.push_back(extents);
        robot_links.push_back(links[i]);
      }
    }
  }

  n_dof = int(robot_links.size());
  ROS_INFO("Finished initializing collision checker.");
  for (int i=0;i<n_dof;i++) {
    vis_pubs.push_back(nh.advertise<visualization_msgs::Marker>( "/robot_marker"+std::to_string(i), 0 ));
  }
  pub_pt = nh.advertise<visualization_msgs::Marker>("/human_pt",0);

  pub_robot_pt = nh.advertise<visualization_msgs::Marker>("/robot_points",0);

  // max_cart_acc_ = 1.0;
  // min_hrc_distance_ = 0.3;
  // reaction_time = 0.15;
  // dist_dec_=max_cart_acc_*min_hrc_distance_;
  // term1_=std::pow(dist_dec_,2)-2*max_cart_acc_*min_hrc_distance_;
  urdf::Model robo_model;
  robo_model.initParam("robot_description");
  std::string base_frame_ = "world";
  std::string tool_frame = "tip";
  if (!nh.getParam("base_frame", base_frame_))
  {
    ROS_ERROR("%s/base_frame not defined", nh.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  chain_ = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
  ssm_=std::make_shared<ssm15066::DeterministicSSM>(chain_,nh);

}

void ParallelRobotPointClouds::updatePlanningScene(const planning_scene::PlanningScenePtr &planning_scene) {
  for (int idx=0;idx<threads_num_;idx++)
  {
    ROS_INFO_STREAM("planning scene clone "<<idx);
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene));
    queues_.at(idx) = std::vector<std::pair<int,std::vector<double>>>();
  }
}

void ParallelRobotPointClouds::pt_intersection(Eigen::Isometry3f &link_transform, int link_id, int thread_idx) {
  //invert link transform
  Eigen::MatrixXf transformed_pts = link_transform.inverse()*model_->avoid_cart_pts;
  bool is_collision = true;
  assert(model_pts_already_checked.size()==transformed_pts.cols());
  // float pt_min_dist = std::numeric_limits<float>::infinity();
  for (int i = 0; i<transformed_pts.cols();i++) {
    // mtx.lock();
    bool already_checked = model_pts_already_checked[i];
    // mtx.unlock();
    if (already_checked) continue;
    is_collision = true;
    for (int j=0;j<3;j++) {
      //+-grid_spacing_
      if (((-0.5*link_boxes[link_id][j])>transformed_pts.col(i)[j]) || (transformed_pts.col(i)[j]>(0.5*link_boxes[link_id][j]))) {
        is_collision = false;
        break;
      }
    }
    // if (use_iso15066) {
    //   Eigen::Vector3f dist;
    //   dist[0] = transformed_pts.col(i).norm();
    //   dist[1] = (transformed_pts.col(i)-Eigen::Vector3f(0,0,0.5*link_boxes[link_id][2])).norm();
    //   dist[2] = (transformed_pts.col(i)-Eigen::Vector3f(0,0,0.5*link_boxes[link_id][2])).norm();
    //   pt_min_dist = std::min(min_dist,dist.minCoeff());
    // }
    if (is_collision) {
      // displayCollisionPoint(0.05,transformed_pts.col(i));
      model_pts_already_checked[i] = true;
      // mtx.lock();
      th_avoid_ints[thread_idx].insert(std::end(th_avoid_ints[thread_idx]),std::begin(model_->model_points[model_->model_pt_idx[i]].avoidance_intervals_), std::end(model_->model_points[model_->model_pt_idx[i]].avoidance_intervals_));
      // mtx.unlock();
    }
  }
  // return pt_min_dist;
}

void ParallelRobotPointClouds::show_transformed_pts(Eigen::MatrixXf &transformed_pts) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 11000;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  geometry_msgs::Point p;

  for (int i = 0;i<transformed_pts.cols();i++) {
    p.x = transformed_pts.col(i)[0];
    p.y = transformed_pts.col(i)[1];
    p.z = transformed_pts.col(i)[2];
    marker.points.push_back(p);
  }
  pub_pt.publish( marker );
}

void ParallelRobotPointClouds::resetQueue()
{
  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
    queues_.at(idx) = std::vector<std::pair<int,std::vector<double>>>();
  }
}

void ParallelRobotPointClouds::queueUp(const Eigen::VectorXd &q)
{
  if(q.size() != 0)
  {
    std::vector<double> conf(q.size());
    for (unsigned idx=0;idx<q.size();idx++)
      conf.at(idx)=q(idx);


    queues_.at(thread_iter_).push_back(std::pair<int,std::vector<double>>(0,conf));
    thread_iter_ ++;
    if (thread_iter_>=threads_num_)
      thread_iter_=0;
  }
  else
    throw std::invalid_argument("q is empty");
}

void ParallelRobotPointClouds::checkAllQueues(std::vector<Eigen::Vector3f> &combined_avoidance_intervals, float &last_pass_time)
{
  // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  model_->ext_mutex.lock();
  model_pts_already_checked = std::vector<bool>(model_->model_pt_idx.size(),false);
  stop_check_=false;
  avoid_ints.clear();
  // threads.resize(threads_num_);
  for (int idx=0;idx<threads_num_;idx++)
  {
    th_avoid_ints[idx].clear();
    if (queues_.at(idx).size()>0) {
        threads.at(idx)=std::thread(&ParallelRobotPointClouds::collisionThread,this,idx);
    } else {
      break;
    }
  }
  //wait for threads to finish
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
    avoid_ints.insert(std::end(avoid_ints), std::begin(th_avoid_ints[idx]),std::end(th_avoid_ints[idx]));
  }

  model_->ext_mutex.unlock();
  //merge the intervals?
  combined_avoidance_intervals.clear();
  // ROS_INFO_STREAM("avoid_ints size:"<<avoid_ints.size());
  avoidance_intervals::merge_intervals(avoid_ints,combined_avoidance_intervals, last_pass_time);

  // return *std::min_element(min_dists.begin(), min_dists.end());

  // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds."<<combined_avoidance_intervals.size());
}

//check for collision between robot bounding shaps and the cloud of points with avoidance intervals for configurations between nodes
void ParallelRobotPointClouds::collisionThread(int thread_idx)
{
  std::vector<Eigen::Isometry3f> link_transforms(n_dof); 

  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::pair<int,std::vector<double>>>& queue=queues_.at(thread_idx);
  for (const std::pair<int,std::vector<double>>& queue_element: queue)
  {
    const std::vector<double>& configuration = queue_element.second;
    if (stop_check_)
    {
      break;
    }
    assert(configuration.size()>0);

    bool success = true; //bool to set false if path can not be found
    state->setVariablePositions(configuration);

    // for (int i=0;i<n_dof;i++) {
    //     displayRobot(i,link_boxes[i],link_transforms[i].translation(),Eigen::Quaternionf(link_transforms[i].rotation()));
    // }
    
    // std::cout<<"press enter\n";
    // std::cin.ignore();
    float min_dist = std::numeric_limits<float>::infinity();
    for (int i=0;i<n_dof;i++) {
      Eigen::Isometry3f offset_transform;
      offset_transform.setIdentity();
      offset_transform.translate(link_bb_offsets[i]);
      link_transforms[i] = Eigen::Isometry3f(state->getGlobalLinkTransform(robot_links[i]).cast<float>()*offset_transform);
      pt_intersection(link_transforms[i], i, thread_idx);
      // min_dist = std::min(min_dist,pt_intersection(link_transforms[i], i, thread_idx));
      // if (thread_idx==0) displayRobot(i,link_boxes[i],link_transforms[i].translation(),Eigen::Quaternionf(link_transforms[i].rotation()));
      // if (thread_idx==0) {
      //   clearRobot();
      //   // ROS_INFO_STREAM("link:"<<i<<",side:"<<link_boxes[i].side.transpose());
      //   show_transformed_pts(tmp);
      //   displayRobot(i,link_boxes[i],Eigen::Vector3f(0,0,0),Eigen::Quaternionf(1,0,0,0));
      //   std::cout<<"press enter\n";
      //   std::cin.ignore();
      // }
    }
    // min_dists[queue_element.first] = min_dist;

    // ROS_INFO_STREAM("push enter");
    // std::cin.ignore();
  }
}




ParallelRobotPointClouds::~ParallelRobotPointClouds()
{
  ROS_DEBUG("closing collision threads");
  stop_check_=true;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  ROS_DEBUG("collision threads closed");
}

void ParallelRobotPointClouds::queueConnection(const Eigen::VectorXd& configuration1,
                                                    const Eigen::VectorXd& configuration2)
{
  
  double distance = (configuration2 - configuration1).norm();
  int num_steps = ceil(distance/min_distance_);
  min_dists.resize(num_steps+1);
  tangential_vels.resize(num_steps+1);
  int num_confs_per_thread = ceil(double(num_steps+1)/double(threads_num_));
  Eigen::VectorXd conf(configuration1.size());
  int thread_iter_ = 0;
  int conf_iter_ = 0;
  for (int i=0;i<num_steps+1;i++) {
    conf = configuration1+(configuration2-configuration1)*i/num_steps;
    std::vector<double> conf_vec(conf.size());
    for (unsigned idx=0;idx<conf.size();idx++) conf_vec.at(idx) = conf(idx);
    queues_.at(thread_iter_).push_back(std::pair<int,std::vector<double>>(conf_iter_,conf_vec));
    conf_iter_++;
    if (queues_.at(thread_iter_).size()>num_confs_per_thread) thread_iter_++;
  }
  // queueUp(configuration1);
  // queueUp(configuration2);
  // if (distance < min_distance_)
  //   return;


  // Eigen::VectorXd conf(configuration1.size());
  // double n = 2;

  // while (distance > n * min_distance_)
  // {
  //   for (double idx = 1; idx < n; idx += 2)
  //   {
  //     conf = configuration1 + (configuration2 - configuration1) * idx / n;
  //     queueUp(conf);
  //   }
  //   n *= 2;
  // }
}

void ParallelRobotPointClouds::checkPath(const Eigen::VectorXd& configuration1,
                                              const Eigen::VectorXd& configuration2, 
                                              std::vector<Eigen::Vector3f> &avoid_ints,
                                              float &last_pass_time)
{
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  resetQueue();
  // if (!check(configuration1))
  //   return false;
  // if (!check(configuration2))
  //   return false;
  queueConnection(configuration1,configuration2);
  checkAllQueues(avoid_ints,last_pass_time);
  // float min_dist = checkAllQueues(avoid_ints,last_pass_time);

  // stop_check_=false;
  // GenerateAllConfigPts();
  // quequeCollisionPtThread(avoid_ints,last_pass_time);
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds.");
  // PATH_COMMENT_STREAM("continue?");
  // std::cin.ignore();
  // return min_dist;
}

void ParallelRobotPointClouds::displayRobot(int i,Eigen::Vector3f sides,Eigen::Vector3f pos, Eigen::Quaternionf quat) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = i+10000;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.scale.x = sides[0];
  marker.scale.y = sides[1];
  marker.scale.z = sides[2];
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  vis_pubs[i].publish( marker );
}

void ParallelRobotPointClouds::clearRobot(void) {
  visualization_msgs::Marker marker;
  for (int i = 0;i<n_dof;i++) {
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i+10000;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETE;
    vis_pubs[i].publish( marker );
  }
}

void ParallelRobotPointClouds::displayCollisionPoint(float radius,Eigen::Vector3f pos) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 11000;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 2*radius;
  marker.scale.y = 2*radius;
  marker.scale.z = 2*radius;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker.points.push_back(p);
  p.x = 1;
  p.y = 0;
  p.z = 0;
  marker.points.push_back(p);
  p.x = 0;
  p.y = 2;
  p.z = 0;
  marker.points.push_back(p);
  p.x = 0;
  p.y = 0;
  p.z = 0.2;
  marker.points.push_back(p);
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  marker.points.push_back(p);
  pub_pt.publish( marker );
}

double ParallelRobotPointClouds::checkISO15066(const Eigen::VectorXd& configuration1,
                                              const Eigen::VectorXd& configuration2, double length, float t1, float t2, unsigned int nsteps, float &min_human_dist) {
    float nominal_time = t2-t1;
    min_human_dist = 1.0;
    // std::cout<<int(model_->joint_seq.size());
    if (model_->joint_seq.empty()) return t2;
    double model_t_step = 0.1;
    // ROS_INFO_STREAM("joint seq:"<<model_->joint_seq.size()<<","<<model_t_step);
    Eigen::VectorXd nominal_velocity= (configuration2 - configuration1)/nominal_time;
    double cost=0;
    double inv_nsteps = 1.0 / nsteps;
    double segment_time = nominal_time / nsteps;
    for (unsigned int istep = 0; istep < nsteps+1; istep++)
    { 
      int step_num = int((t1+(double)istep*segment_time)/model_t_step);
      if (step_num<model_->joint_seq.size()) {
        ssm_->setPointCloud(model_->joint_seq[step_num].second);
      } else {
        // ssm_->setPointCloud(Eigen::Matrix3Xd());
        ssm_->setPointCloud(model_->joint_seq.back().second);
      }
      Eigen::VectorXd q = configuration1 + (configuration2 - configuration1) * inv_nsteps * (double)istep;

      double scaling=ssm_->computeScaling(q,nominal_velocity);
      min_human_dist = (float)scaling;
      // std::cout<<","<<min_human_dist<<",";

      // std::cout<<q.transpose()<<", scale:"<<scaling<<", "<<model_t_step<<std::endl;
      double max_seg_time = segment_time/(scaling+1e-6);
      if (scaling<0.1) {
        for (int i=1;i<30;i++) {
          if (step_num+i<model_->joint_seq.size()) {
            ssm_->setPointCloud(model_->joint_seq[step_num+i].second);
            scaling=ssm_->computeScaling(q,nominal_velocity);
            if (scaling>0.1) { 
              max_seg_time = (segment_time+double(i)*0.1)/scaling;
              min_human_dist = (float)scaling;
              break;
            }
          } else {
            max_seg_time = segment_time;
            break;
          }
        }
      }
      cost+=max_seg_time; //avoid division by zero
    }
    // ROS_INFO_STREAM("ssm cost:"<<cost);
    // std::cout<<std::endl;

    return cost+(double)t1;
    // state->setVariablePositions(configuration1);
    // Eigen::MatrixXd jacobian = state->getJacobian(joint_model_group);
    // Eigen::MatrixXd robot_cart_vels1 = jacobian.colwise()*max_q_dot_;
    // //check for robot approaching human at conf1
    // //get vectors between robot joint and human joints
    // double speed_scale = 1.0;
    // double robot_relative_velocity_t1 = 0.0;
    // for (int i=0;i<robot_links.size();i++) {
    //   Eigen::Vector3d joint_loc = state->getGlobalLinkTransform(robot_links[i]).translation();
    //   for (int j=0;j<human_joints_t1.size();j++) {
    //     Eigen::Vector3d approach_vec = human_joints_t1[j].cast<double>()-joint_loc;
    //     double dist = approach_vec.norm();
    //     // double vmax_=std::sqrt(term1_+2.0*max_cart_acc_*dist)-dist_dec_;
    //     robot_relative_velocity_t1 = std::max(robot_relative_velcotiy_t1,robot_cart_vels1.colwise().dot(approach_vec)/dist);
    //     // speed_scale = std::min(speed_scale, vmax_/robot_relative_velocity);
    //   }
    // }
    // state->setVariablePositions(configuration2);
    // jacobian = state->getJacobian(joint_model_group);
    // Eigen::MatrixXd robot_cart_vels2 = jacobian.colwise()*max_q_dot_;
    // double robot_relative_velocity_t2 = 0.0;
    // human_joints_t2 = model->joint_seq[int(t2/model_t_step)];
    // for (int i=0;i<robot_links.size();i++) {
    //   Eigen::Vector3d joint_loc = state->getGlobalLinkTransform(robot_links[i]).translation();
    //   for (int j=0;j<human_joints_t2.size();j++) {
    //     Eigen::Vector3d approach_vec = human_joints_t2[j].cast<double>()-joint_loc;
    //     double dist = approach_vec.norm();
    //     // double vmax_=std::sqrt(term1_+2.0*max_cart_acc_*dist)-dist_dec_;
    //     robot_relative_velocity_t2 = std::max(robot_relative_velocity_t2,robot_cart_vels2.colwise().dot(approach_vec)/dist);
    //     // speed_scale = std::min(speed_scale, vmax_/robot_relative_velocity);
    //   }
    // }

    // if (robot_relative_velocity_t2>0) {
      
    // } else if (robot_relative_velocity_t1>0) {

    // } else {
    //   speed_scale = 1.0;
    // }
}

}