#include <graph_core/parallel_robot_point_clouds.h>

namespace pathplan
{
ParallelRobotPointClouds::ParallelRobotPointClouds(const planning_scene::PlanningScenePtr &planning_scene,
                                                               const std::string& group_name, const avoidance_intervals::modelPtr& model,
                                                               const int& threads_num,
                                                               const double& min_distance):
  model_(model)
{
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
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene));
    queues_.push_back(std::vector<std::vector<double>>());
  }

  req_.distance=false;
  req_.group_name=group_name;
  req_.verbose=false;
  req_.contacts=false;
  req_.cost=false;

  std::vector<std::vector<shapes::ShapeConstPtr>> link_shapes;
  // std::vector<int> vert_indices;
  // std::vector<double> verts;
  // unsigned int* tri_ptr;
  // double* vert_ptr;
  // double num_triangles;
  // double num_vertices;
  // fcl::Transform3d link_transform;
  // std::vector<fcl::Triangle> fcl_link_triangles;
  // std::vector<fcl::Vector3d> fcl_link_vertices;
  // std::vector<std::vector<fcl::Triangle>> fcl_robot_triangles;
  // std::vector<std::vector<fcl::Vector3d>> fcl_robot_vertices;
  robot_state::RobotState robo_state(planning_scene->getCurrentState());
  links = robo_state.getJointModelGroup(group_name)->getLinkModels();
  link_shapes.resize(links.size());
  for (int i=0;i<int(links.size());i++) {
    link_shapes[i] = links[i]->getShapes();
    if (link_shapes[i].size()>0){
      // if (link_shapes[i].at(0)->type==shapes::MESH) {
      //   const shapes::Shape* shape(link_shapes[i].at(0).get());
      //   const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape);
      //   tri_ptr = mesh->triangles;
      //   vert_ptr = mesh->vertices;
      //   num_triangles = mesh->triangle_count;
      //   num_vertices = mesh->vertex_count;
      //   fcl_link_vertices.clear();
      //   for (int j=0;j<num_vertices;j++) {
      //       verts.clear();
      //       for (int k=0;k<3;k++) {
      //           verts.push_back(*vert_ptr);
      //           vert_ptr++;
      //       }
      //       fcl::Vector3d tmp_vertex(verts[0],verts[1],verts[2]);
      //       fcl_link_vertices.push_back(tmp_vertex);
      //   }
      //   fcl_robot_vertices.push_back(fcl_link_vertices);
      //   fcl_link_triangles.clear();
      //   for (int j=0;j<num_triangles;j++) {
      //       vert_indices.clear();
      //       for (int k=0;k<3;k++) {
      //           vert_indices.push_back(*tri_ptr);
      //           tri_ptr++;
      //       }
      //       fcl::Triangle tmp_triangle(vert_indices[0],vert_indices[1],vert_indices[2]);
      //       fcl_link_triangles.push_back(tmp_triangle);
      //   }
      //   fcl_robot_triangles.push_back(fcl_link_triangles);
      // }
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
    }
  }
  ROS_INFO("Finished initializing collision checker.");
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

  std::vector<fcl::Transform3f> link_transforms; 

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
    fcl::CollisionRequest<float> req(true);
    fcl::CollisionResult<float> res;

    bool success = true; //bool to set false if path can not be found
    state->setVariablePositions(configuration);
    
    //convert robot to point cloud, or
    //determine which model points the robot is in collision with at configuration
    //probably need to write a function to descritize robot bounding boxes or cylinders to point clouds
    for (int i=1;i<n_dof;i++) {
        //offset transform relates robot link bounding box to origin of link
        Eigen::Isometry3f offset_transform;
        offset_transform.setIdentity();
        offset_transform.translate(link_bb_offsets[i]);
        //getGlobalLinkTransform relates link origin to work frame
        link_transforms[i] = fcl::Transform3f(state->getGlobalLinkTransform(links[i]).cast<float>()*offset_transform);
        //generate collision object for robot link at config
        std::shared_ptr<fcl::CollisionGeometry<float>> link_box_ptr(&link_boxes[i]);
        fcl::CollisionObjectf* robot = new fcl::CollisionObjectf(link_box_ptr, link_transforms[i]);
        fcl::collide(robot,model_->point_cloud_,req,res);
        //get collision points out of res and append them to a vector
        std::vector<fcl::Contact<float>> collisionContacts;
        res.getContacts(collisionContacts);
        for (fcl::Contactf pt:collisionContacts) {
          collision_points.push_back(pt.pos);
        }
    }
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
}

}