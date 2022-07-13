#include <graph_core/parallel_robot_point_clouds.h>

namespace pathplan
{
ParallelRobotPointClouds::ParallelRobotPointClouds(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene,
                                                               const std::string& group_name, const avoidance_intervals::modelPtr& model,
                                                               const int& threads_num,
                                                               const double& min_distance):
  model_(model), threads_num_(threads_num), nh(node_handle)
{

  ROS_INFO_STREAM("create parallel point clouds with "<<threads_num_<<" threads");
  // ROS_INFO_STREAM("press enter");
  // std::cin.ignore();
  min_distance_ = min_distance;
  group_name_ = group_name;

  if (threads_num<=0)
    throw std::invalid_argument("number of thread should be positive");

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  threads.resize(threads_num_);
  for (int idx=0;idx<threads_num_;idx++)
  {
    ROS_INFO_STREAM("planning scene clone "<<idx);
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene));
    queues_.push_back(std::vector<std::vector<double>>());
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
        PATH_COMMENT_STREAM("link name "<<link_name);
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
    queues_.at(idx).clear();
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
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  model_pts_already_checked = std::vector<bool>(model_->model_points.size(),false);
  stop_check_=false;
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (queues_.at(idx).size()>0)
      threads.at(idx)=std::thread(&ParallelRobotPointClouds::collisionThread,this,idx);
  }
  //wait for threads to finish
  for (int idx=0;idx<threads_num_;idx++)
  {
    if (threads.at(idx).joinable())
      threads.at(idx).join();
  }
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  // ROS_INFO_STREAM("It took me " << time_span.count() << " seconds.");

  //sort collision points
  //need to sort collision points in order of increasing x, then y, then z,
  //there will be many duplicate points since the points for each link were merged
  //and links overlap at joints
  std::sort(collision_points.begin(),collision_points.end(),avoidance_intervals::eigen_sort_rows_by_columns);
  combined_avoidance_intervals.clear();
  std::vector<Eigen::Vector3f> avoid_ints;
  std::vector<Eigen::Vector3f> pt_avoid_ints;
  for (int i = 0; i<collision_points.size();i++) {
      if (i<collision_points.size()-1) {
          while (collision_points[i].isApprox(collision_points[i+1])) {
              i++;
          }
      }
      //look up avoidance intervals for each point robot occupies
      pt_avoid_ints = model_->model_points[model_->cart_to_model(collision_points[i])].avoidance_intervals_;
      avoid_ints.insert(std::end(avoid_ints),std::begin(pt_avoid_ints), std::end(pt_avoid_ints));
  }
  //merge the intervals?
  avoidance_intervals::merge_intervals(avoid_ints,combined_avoidance_intervals, last_pass_time);
}

//check for collision between robot bounding shaps and the cloud of points with avoidance intervals for configurations between nodes
void ParallelRobotPointClouds::collisionThread(int thread_idx)
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
      link_transforms[i] = fcl::Transform3f(state->getGlobalLinkTransform(robot_links[i]).cast<float>()*offset_transform);
    }

    //convert robot to point cloud, or
    //determine which model points the robot is in collision with at configuration
    //probably need to write a function to descritize robot bounding boxes or cylinders to point clouds
    // for (int i=1;i<n_dof;i++) {
    //     //offset transform relates robot link bounding box to origin of link
    //     Eigen::Isometry3f offset_transform;
    //     offset_transform.setIdentity();
    //     offset_transform.translate(link_bb_offsets[i]);
    //     //getGlobalLinkTransform relates link origin to work frame
    //     fcl::Transform3f link_transform = fcl::Transform3f(state->getGlobalLinkTransform(links[i]).cast<float>()*offset_transform);
    //     //generate collisionavoidance_intervals_ object for robot link at config
    //     // std::shared_ptr<fcl::CollisionGeometry<float>> link_box_ptr(&link_boxes[i]);
    //     // fcl::CollisionObjectf* robot = new fcl::CollisionObjectf(link_box_ptr, link_transform);
    //     fcl::Transform3f obj_pose = fcl::Transform3f::Identity();
    //     //get collision points out of res and append them to a vector
    //     for (int j:model_pt_idx) {
    //     // for (avoidance_intervals::model_point pt:model_->model_points) {
    //       model_point pt = model_->model_points[j];
    //       if (pt.avoidance_intervals_.empty()) continue;
    //       obj_pose = fcl::Transform3f::Identity();
    //       obj_pose.translation() = fcl::Vector3f(pt.position[0],pt.position[1],pt.position[2]);
    //       fcl::collide(&link_boxes[i],link_transform,&model_->point_sphere,obj_pose,req,res);
    //       if (res.isCollision()) {
    //         collision_points.push_back(Eigen::Vector3f(pt.position[0],pt.position[1],pt.position[2]));
    //       }
    //     }
    // }

    for (int j:model_->model_pt_idx) {
      if (model_pts_already_checked[j]) {
        continue;
      }
      fcl::Transform3f obj_pose = fcl::Transform3f::Identity();
      //get collision points out of res and append them to a vector
      // for (avoidance_intervals::model_point pt:model_->model_points) {
      // avoidance_intervals::model_point pt = model_->model_points[j];
      // displayCollisionPoint(0.05,model_->model_points[j].position);
      // ROS_INFO_STREAM("push enter"<<model_->model_points[j].position.transpose());
      // std::cin.ignore();
      if (model_->model_points[j].avoidance_intervals_.empty()) continue;
      // PATH_COMMENT_STREAM("avoid ints "<<pt.avoidance_intervals_.size());
      obj_pose = fcl::Transform3f::Identity();
      obj_pose.translation() = model_->model_points[j].position;//fcl::Vector3f(pt.position[0],pt.position[1],pt.position[2]);

      for (int i=1;i<n_dof;i++) {
        // displayRobot(i,link_boxes[i].side,link_transforms[i].translation(),Eigen::Quaternionf(link_transforms[i].rotation()));
        //offset transform relates robot link bounding box to origin of link
        //getGlobalLinkTransform relates link origin to work frame
        //generate collisionavoidance_intervals_ object for robot link at config
        // std::shared_ptr<fcl::CollisionGeometry<float>> link_box_ptr(&link_boxes[i]);
        // fcl::CollisionObjectf* robot = new fcl::CollisionObjectf(link_box_ptr, link_transform);

        fcl::CollisionResult<float> res;
        fcl::collide(&link_boxes[i],link_transforms[i],&model_->point_sphere,obj_pose,req,res);

        if (res.isCollision()) {
          model_pts_already_checked[j] = true;
          PATH_COMMENT_STREAM("avoid ints collision");
          collision_points.push_back(model_->model_points[j].position);
          break;
        }
      }
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
  resetQueue();
  // if (!check(configuration1))
  //   return false;
  // if (!check(configuration2))
  //   return false;
  queueConnection(configuration1,configuration2);
  checkAllQueues(avoid_ints,last_pass_time);

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