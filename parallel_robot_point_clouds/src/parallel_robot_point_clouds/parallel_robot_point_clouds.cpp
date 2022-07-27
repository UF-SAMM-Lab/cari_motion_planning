#include <parallel_robot_point_clouds/parallel_robot_point_clouds.h>

namespace pathplan
{
ParallelRobotPointClouds::ParallelRobotPointClouds(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene,
                                                               const std::string& group_name, const avoidance_intervals::modelPtr& model,
                                                               const int& threads_num,
                                                               const double& min_distance,
                                                               const double& grid_spacing):
  model_(model), threads_num_(threads_num), nh(node_handle),grid_spacing_(grid_spacing)
{

  ROS_INFO_STREAM("create parallel point clouds with "<<threads_num_<<" threads");
  // ROS_INFO_STREAM("press enter");
  // std::cin.ignore();
  min_distance_ = min_distance;
  group_name_ = group_name;
  fcl_check = true;
  if (!nh.getParam("fcl_collision_check",fcl_check))
  {
    ROS_DEBUG("fcl_collision_check is not set, default=0.04");
    fcl_check=true;
  }

  if (threads_num<=0)
    throw std::invalid_argument("number of thread should be positive");

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  threads.resize(threads_num_);
  queues_.resize(threads_num_);
  for (int idx=0;idx<threads_num_;idx++)
  {
    ROS_INFO_STREAM("planning scene clone "<<idx);
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene));
    queues_.at(idx) = std::vector<std::vector<double>>();
  }

  ROS_INFO_STREAM("parallel point clouds 1");
  req_.distance=false;
  req_.group_name=group_name;
  req_.verbose=false;
  req_.contacts=false;
  req_.cost=false;

  std::vector<std::vector<shapes::ShapeConstPtr>> link_shapes;

  robot_state::RobotState robo_state(planning_scene->getCurrentState());
  links = robo_state.getJointModelGroup(group_name)->getLinkModels();
  link_shapes.resize(links.size());
  ROS_INFO_STREAM("num links"<<link_shapes.size());
  for (int i=0;i<int(links.size());i++) {
    std::string link_name = links[i]->getName();
    if (link_name.substr(link_name.length()-4) == "link") {
      link_shapes[i] = links[i]->getShapes();
      if (link_shapes[i].size()>0){
        ROS_INFO_STREAM("link name "<<link_name);
        const Eigen::Vector3f& extents_tmp = links[i]->getShapeExtentsAtOrigin().cast<float>();
        Eigen::Vector3f extents = extents_tmp;
        const Eigen::Vector3f& offset_tmp = links[i]->getCenteredBoundingBoxOffset().cast<float>();
        Eigen::Vector3f offset = offset_tmp;
        link_bb_offsets.push_back(offset);
        // Eigen::Isometry3d transform;
        // transform.setIdentity();
        // transform.translate(offset);
        // moveit::core::AABB aabb;
        // aabb.extendWithTransformedBox(transform,extents);
        fcl::Boxf link_box(extents[0],extents[1],extents[2]);
        link_raw_pts.push_back(boxPts(extents));
        link_boxes.push_back(link_box);
        robot_links.push_back(links[i]);
      }
    }
  }
  ROS_INFO("Finished initializing collision checker.");
  for (int i=0;i<6;i++) {
    vis_pubs.push_back(nh.advertise<visualization_msgs::Marker>( "/robot_marker"+std::to_string(i), 0 ));
  }
  pub_pt = nh.advertise<visualization_msgs::Marker>("/human_pt",0);

  pub_robot_pt = nh.advertise<visualization_msgs::Marker>("/robot_points",0);
}

Eigen::MatrixXf ParallelRobotPointClouds::boxPts(Eigen::Vector3f extents) {
  double grid_spacing = grid_spacing_*0.7;
  int z_lim = ceil(extents[2]/grid_spacing);
  int y_lim = ceil(extents[1]/grid_spacing);
  int x_lim = ceil(extents[0]/grid_spacing);
  Eigen::MatrixXf pts(3,(z_lim+1)*(y_lim+1)*(x_lim+1));
  Eigen::Vector3f mid_pt(x_lim,y_lim,z_lim);
  mid_pt *= 0.5*grid_spacing;
  int l = 0;
  for (int k=0;k<z_lim;k++) {
     for (int j=0;j<y_lim;j++) {
      for (int i=0;i<x_lim;i++) {
        pts.col(l) = Eigen::Vector3f(i,j,k)*grid_spacing-mid_pt;
        l++;
      }
     }   
  }
  pts.conservativeResize(3,l);
  return pts;
}

void ParallelRobotPointClouds::pt_intersection(fcl::Transform3f &link_transform, int link_id, int thread_id) {
  //invert link transform
  Eigen::MatrixXf transformed_pts = link_transform.inverse()*model_->avoid_cart_pts;
  bool is_collision = true;
  assert(model_pts_already_checked.size()==transformed_pts.cols());
  for (int i = 0; i<transformed_pts.cols();i++) {
    // mtx.lock();
    bool already_checked = model_pts_already_checked[i];
    // mtx.unlock();
    if (already_checked) continue;
    is_collision = true;
    for (int j=0;j<3;j++) {
      if (((-0.5*link_boxes[link_id].side[j]-grid_spacing_)>transformed_pts.col(i)[j]) || (transformed_pts.col(i)[j]>(0.5*link_boxes[link_id].side[j]+grid_spacing_))) {
        is_collision = false;
        break;
      }
    }
    if (is_collision) {
      model_pts_already_checked[i] = true;
      mtx.lock();
      avoid_ints.insert(std::end(avoid_ints),std::begin(model_->model_points[model_->model_pt_idx[i]].avoidance_intervals_), std::end(model_->model_points[model_->model_pt_idx[i]].avoidance_intervals_));
      mtx.unlock();
    }
  }
  // return transformed_pts;
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

void ParallelRobotPointClouds::sort_reduce_link_pts(std::vector<Eigen::Vector3f> &link_pts) {
  if (link_pts.empty()) {
    ROS_INFO_STREAM("error, no link points");
    return;
  } 
  std::sort(link_pts.begin(),link_pts.end(),avoidance_intervals::eigen_sort_rows_by_columns);
  std::vector<Eigen::Vector3f>::iterator prev_it = link_pts.begin();
  std::vector<Eigen::Vector3f>::iterator it = prev_it+1;
  while (true) {
    bool same = true;
    for (int i=0;i<3;i++) {
      if (prev(it,1)[i]!=it[i]) {
        same=false;
        break;
      }
    }
    if (same) {
      link_pts.erase(it);
    }
    if (it == link_pts.end()) break;
    // prev_it = it;
    it++;
  }
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
    queues_.at(idx) = std::vector<std::vector<double>>();
  }
}

void ParallelRobotPointClouds::queueUp(const Eigen::VectorXd &q)
{
  if(q.size() != 0)
  {
    std::vector<double> conf(q.size());
    for (unsigned idx=0;idx<q.size();idx++)
      conf.at(idx)=q(idx);


    queues_.at(thread_iter_).push_back(conf);
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
    if (queues_.at(idx).size()>0) {
      if (fcl_check) {
        threads.at(idx)=std::thread(&ParallelRobotPointClouds::collisionThread,this,idx);
      } else {
        threads.at(idx)=std::thread(&ParallelRobotPointClouds::collisionThread2,this,idx);
      }

    } else {
      break;
    }
  }
  //wait for threads to finish
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }

  model_->ext_mutex.unlock();
  //merge the intervals?
  combined_avoidance_intervals.clear();
  // ROS_INFO_STREAM("avoid_ints size:"<<avoid_ints.size());
  avoidance_intervals::merge_intervals(avoid_ints,combined_avoidance_intervals, last_pass_time);

  // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds."<<combined_avoidance_intervals.size());
}

void ParallelRobotPointClouds::GenerateAllConfigPts(void)
{

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  link_pts.clear();

  for (int idx=0;idx<threads_num_;idx++)
  {
    if (queues_.at(idx).size()>0)
      threads.at(idx)=std::thread(&ParallelRobotPointClouds::generatePtsThread,this,idx);
  }
  //wait for threads to finish
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }

  link_pts_vec.clear();
  for (int i=0;i<link_pts.size();i++) {
    for (int j=0;j<link_pts[i].cols();j++) link_pts_vec.push_back(link_pts[i].col(j));
  }

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  ROS_INFO_STREAM("generating points took " << time_span.count() << " seconds.");

  sort_reduce_link_pts(link_pts_vec);

  std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
  ROS_INFO_STREAM("sorting took " << time_span2.count() << " seconds.");
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds."<<combined_avoidance_intervals.size());
}

void ParallelRobotPointClouds::generatePtsThread(int thread_idx)
{
  std::vector<Eigen::Isometry3f> link_transforms(n_dof); 

  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::vector<double>>& queue=queues_.at(thread_idx);
  for (const std::vector<double>& configuration: queue)
  {
    if (stop_check_)
    {
      break;
    }
    assert(configuration.size()>0);

    state->setVariablePositions(configuration);
    
    // std::vector<Eigen::MatrixXf> link_pts(n_dof);
    int num_link_pts = 0;
    for (int i=0;i<n_dof;i++) {
      Eigen::Isometry3f offset_transform;
      offset_transform.setIdentity();
      offset_transform.translate(link_bb_offsets[i]);
      link_transforms[i] = Eigen::Isometry3f(state->getGlobalLinkTransform(robot_links[i]).cast<float>()*offset_transform);
      Eigen::Matrix3f tmp_transform(link_transforms[i].rotation());
      // displayRobot(i,link_boxes[i].side,link_transforms[i].translation(),Eigen::Quaternionf(link_transforms[i].rotation()));
      //transform robot points
      // ROS_INFO_STREAM("thread:"<<thread_idx<<"link "<<i<<", link raw pts size:"<<link_raw_pts[i].rows()<<","<<link_raw_pts[i].cols());

      Eigen::MatrixXf oversampled_pts = (link_transforms[i].rotation()*link_raw_pts[i]).colwise()+link_transforms[i].translation();

      // ROS_INFO_STREAM("i:"<<i<<" link transform size:"<<oversampled_pts.size());
      //downsample the link pts, sort and remove duplicates
      avoidance_intervals::downsample_points(oversampled_pts, grid_spacing_);
      link_pts.push_back(oversampled_pts);

        // link_pts[i] = link_pts[i].colwise()+link_transforms[i].translation();
        // ROS_INFO_STREAM("link transform size:"<<link_pts[i].rows()<<","<<link_pts[i].cols());
      num_link_pts += link_raw_pts[i].cols();

    }
  }

}

void ParallelRobotPointClouds::quequeCollisionPtThread(std::vector<Eigen::Vector3f> &combined_avoidance_intervals, float &last_pass_time) {
  int pts_per_thread = ceil((float)model_->model_pt_idx.size()/(float)threads_num_);

  threads.clear();
  threads.resize(threads_num_);
  avoid_ints.clear();
  for (int j:model_->model_pt_idx) {
    std::cout<<model_->model_points[j].position;
  }
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (queues_.at(idx).size()>0)
      threads.at(idx)=std::thread(&ParallelRobotPointClouds::collisionThreadPts,this,idx, idx*pts_per_thread, std::min((idx+1)*pts_per_thread,int(model_->model_pt_idx.size())));
  }
  //wait for threads to finish
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }


  //merge the intervals?
  combined_avoidance_intervals.clear();
  // ROS_INFO_STREAM("avoid_ints size:"<<avoid_ints.size());
  avoidance_intervals::merge_intervals(avoid_ints,combined_avoidance_intervals, last_pass_time);

}

bool vectorEqual(Eigen::Vector3f vec1, Eigen::Vector3f vec2) {
  for (int i=0;i<vec1.size();i++){
    if (round(vec1[i]*1000)!=round(vec2[i]*1000)) {
      return false;
    }
  }
  return true;
}

//check for collision between robot bounding shaps and the cloud of points with avoidance intervals for configurations between nodes
void ParallelRobotPointClouds::collisionThreadPts(int thread_idx, int start, int end)
{
    int last_pos = 0;

    for (int i=start;i<end;i++) {
      if (stop_check_) break;
      int j = model_->model_pt_idx[i];
      //get collision points out of res and append them to a vector
      if (model_->model_points[j].avoidance_intervals_.empty()) continue;
      Eigen::Vector3f pt = model_->model_points[j].position;//fcl::Vector3f(pt.position[0],pt.position[1],pt.position[2]);

      for (int k=last_pos;k<link_pts_vec.size();k++) {
        if (stop_check_) break;
        if (vectorEqual(link_pts_vec[k],pt)) {
          mtx.lock();
          avoid_ints.insert(std::end(avoid_ints),std::begin(model_->model_points[j].avoidance_intervals_), std::end(model_->model_points[j].avoidance_intervals_));
          mtx.unlock();
          last_pos = k;
          break;
        }

      }
      
    }  
}

//check for collision between robot bounding shaps and the cloud of points with avoidance intervals for configurations between nodes
void ParallelRobotPointClouds::collisionThread(int thread_idx)
{
  std::vector<fcl::Transform3f> link_transforms; 

  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::vector<double>>& queue=queues_.at(thread_idx);
  for (const std::vector<double>& configuration: queue)
  {
    if (stop_check_)
    {
      break;
    }
    assert(configuration.size()==n_dof);

    //some variables to store request/results of collision check between robot shapes and avoidance point cloud
    fcl::CollisionRequest<float> req;
    req.enable_contact = false;
    req.enable_cost = false;

    bool success = true; //bool to set false if path can not be found
    state->setVariablePositions(configuration);
    
    for (int i=0;i<n_dof;i++) {
      Eigen::Isometry3f offset_transform;
      offset_transform.setIdentity();
      offset_transform.translate(link_bb_offsets[i]);
      link_transforms.push_back(fcl::Transform3f(state->getGlobalLinkTransform(robot_links[i]).cast<float>()*offset_transform));

    }

    for (int k=0;k<model_->model_pt_idx.size();k++) {
      int j = model_->model_pt_idx[k];
      bool already_checked = model_pts_already_checked[k];
      if (already_checked) continue;
      fcl::Transform3f obj_pose = fcl::Transform3f::Identity();
      //get collision points out of res and append them to a vector
      if (model_->model_points[j].avoidance_intervals_.empty()) continue;
      // PATH_COMMENT_STREAM("avoid ints "<<pt.avoidance_intervals_.size());
      obj_pose.translation() = model_->model_points[j].position;//fcl::Vector3f(pt.position[0],pt.position[1],pt.position[2]);

      for (int i=1;i<n_dof;i++) {

        fcl::CollisionResult<float> res;
        //checks intersection of robot boxes and human points
        fcl::collide(&link_boxes[i],link_transforms[i],&model_->point_sphere,obj_pose,req,res);
        
        //try comparison of robot/human points, is model_points[j].position in robot points

        if (res.isCollision()) {
          model_pts_already_checked[k] = true;
          mtx.lock();
          avoid_ints.insert(std::end(avoid_ints),std::begin(model_->model_points[j].combined_avoidance_intervals_), std::end(model_->model_points[j].combined_avoidance_intervals_));
          mtx.unlock();
          break;
        }
      }
    }
    // ROS_INFO_STREAM("push enter");
    // std::cin.ignore();
  }
}


//check for collision between robot bounding shaps and the cloud of points with avoidance intervals for configurations between nodes
void ParallelRobotPointClouds::collisionThread2(int thread_idx)
{

  std::vector<fcl::Transform3f> link_transforms(n_dof); 

  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::vector<double>>& queue=queues_.at(thread_idx);
  for (const std::vector<double>& configuration: queue)
  {
    if (stop_check_)
    {
      break;
    }
    assert(configuration.size()>0);

    bool success = true; //bool to set false if path can not be found
    state->setVariablePositions(configuration);
    
    for (int i=0;i<n_dof;i++) {
      Eigen::Isometry3f offset_transform;
      offset_transform.setIdentity();
      offset_transform.translate(link_bb_offsets[i]);
      link_transforms[i] = fcl::Transform3f(state->getGlobalLinkTransform(robot_links[i]).cast<float>()*offset_transform);
      // Eigen::MatrixXf tmp = 
      pt_intersection(link_transforms[i], i, thread_idx);
      // if (thread_idx==0) {
        // clearRobot();
        // ROS_INFO_STREAM("link:"<<i<<",side:"<<link_boxes[i].side.transpose());
        // displayRobot(i,link_boxes[i].side,link_transforms[i].translation(),Eigen::Quaternionf(link_transforms[i].rotation()));
        // show_transformed_pts(tmp);
        // std::cout<<"press enter\n";
        // std::cin.ignore();
        // displayRobot(i,link_boxes[i].side,Eigen::Vector3f(0,0,0),Eigen::Quaternionf(1,0,0,0));
        // std::cout<<"press enter\n";
        // std::cin.ignore();
      // }
    }

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
  if (distance < min_distance_)
    return;


  Eigen::VectorXd conf(configuration1.size());
  double n = 2;

  while (distance > n * min_distance_)
  {
    for (double idx = 1; idx < n; idx += 2)
    {
      conf = configuration1 + (configuration2 - configuration1) * idx / n;
      queueUp(conf);
    }
    n *= 2;
  }
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

  // stop_check_=false;
  // GenerateAllConfigPts();
  // quequeCollisionPtThread(avoid_ints,last_pass_time);
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds.");
  // PATH_COMMENT_STREAM("continue?");
  // std::cin.ignore();
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
  marker.color.g = 0.0;
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

}