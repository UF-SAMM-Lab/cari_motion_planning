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

#include <avoid_cost_function.h>


namespace pathplan
{

AvoidostFunction::AvoidCostFunction(const ros::NodeHandle &nh):
  GoalCostFunction(),
  nh_(nh)
{

  urdf::Model model;
  model.initParam("robot_description");
  std::string base_frame = "world";
  std::string tool_frame = "tip";
  if (!nh_.getParam("base_frame", base_frame))
  {
    ROS_ERROR("%s/base_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("tool_frame", tool_frame))
  {
    ROS_ERROR("%s/tool_frame not defined", nh_.getNamespace().c_str());
    throw std::invalid_argument("base_frame is not defined");
  }
  if (!nh_.getParam("max_penalty", max_penalty_))
  {
    ROS_ERROR("%s/max_penalty not defined, use 1.0", nh_.getNamespace().c_str());
    max_penalty_=1.0;
  }
  if (!nh_.getParam("max_avoidance_distance", max_distance_))
  {
    ROS_ERROR("%s/max_avoidance_distance not defined, use 1.5 meter", nh_.getNamespace().c_str());
    max_distance_=1.5;
  }
  if (!nh_.getParam("min_avoidance_distance", min_distance_))
  {
    ROS_ERROR("%s/min_avoidance_distance not defined, use 0.5 meter", nh_.getNamespace().c_str());
    min_distance_=0.5;
  }
  if (!nh_.getParam("display_bubbles", plot))
  {
    ROS_ERROR("%s/display_bubbles not defined, use false", nh_.getNamespace().c_str());
    plot=false;
  }
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  chain_ = rosdyn::createChain(model, base_frame, tool_frame, grav);

  if (!nh_.getParam("links", links_))
  {
    ROS_ERROR("%s/links not defined, use all links", nh_.getNamespace().c_str());
    links_=chain_->getLinksName();
  }

  points_.resize(3,0);
  inv_delta_distance_=1.0/(max_distance_-min_distance_);

  if (plot)
  {
    marker_id_=0;
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_cost_function/avoidance_points", 1000);
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id=marker_id_;
    marker.ns = "avoidance";
    marker.header.frame_id="world";
    marker.header.stamp=ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;

    for (unsigned int idx=0;idx<5;idx++)
    {
      marker_pub_.publish(marker);
      ros::Duration(0.01).sleep();
    }
  }

}

void AvoidCostFunction::cleanPoints()
{
  points_.resize(3,0);

  if (!plot)
    return;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.ns = "avoidance";

  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_id_=0;

  marker_pub_.publish(marker);
  ros::Duration(0.1).sleep();

}
void AvoidCostFunction::addPoint(const Eigen::Vector3d &point)
{
  for (int ic=0;ic<points_.cols();ic++)
  {
    if ((points_.col(ic)-point).norm()<1e-4)
      return;
  }
  points_.conservativeResize(3, points_.cols()+1);
  points_.col(points_.cols()-1) = point;

  if (!plot)
    return;

//  visualization_msgs::Marker marker;
//  marker.type = visualization_msgs::Marker::SPHERE;

//  marker.ns = "avoidance";
//  marker.pose.orientation.w=1.0;
//  tf::pointEigenToMsg(point,marker.pose.position);

//  marker.header.frame_id="world";
//  marker.header.stamp=ros::Time::now();
//  marker.action = visualization_msgs::Marker::ADD;
//  marker.id= marker_id_++;

//  marker.scale.x = 2.0*max_distance_;
//  marker.scale.y = 2.0*max_distance_;
//  marker.scale.z = 2.0*max_distance_;

//  marker.color.r = 1;
//  marker.color.g = 0;
//  marker.color.b = 0;
//  marker.color.a = 0.05;


//  marker_pub_.publish(marker);
//  ros::Duration(0.15).sleep();

//  if (min_distance_>0)
//  {
//    marker.scale.x = 2.0*min_distance_;
//    marker.scale.y = 2.0*min_distance_;
//    marker.scale.z = 2.0*min_distance_;
//    marker.id= marker_id_++;
//    marker.color.r = 1;
//    marker.color.g = 0;
//    marker.color.b = 0;
//    marker.color.a = .4;
//    marker_pub_.publish(marker);
//    ros::Duration(0.01).sleep();
//  }

}

void AvoidCostFunction::publishPoints()
{
  if (!plot)
    return;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.ns = "avoidance";
  marker.pose.orientation.w=1.0;


  marker.header.frame_id="world";
  marker.header.stamp=ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id= marker_id_++;

  marker.scale.x = 2.0*max_distance_;
  marker.scale.y = 2.0*max_distance_;
  marker.scale.z = 2.0*max_distance_;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.05;

  for (int ic=0;ic<points_.cols();ic++)
  {
    Eigen::Vector3d point=points_.col(ic);
    geometry_msgs::Point p;
    tf::pointEigenToMsg(point,p);
    marker.points.push_back(p);
    marker.colors.push_back(marker.color);
  }

  marker_pub_.publish(marker);
  ros::Duration(0.15).sleep();

//  if (min_distance_>0)
//  {
//    marker.scale.x = 2.0*min_distance_;
//    marker.scale.y = 2.0*min_distance_;
//    marker.scale.z = 2.0*min_distance_;
//    marker.id= marker_id_++;
//    marker.color.r = 1;
//    marker.color.g = 0;
//    marker.color.b = 0;
//    marker.color.a = .4;
//    marker_pub_.publish(marker);
//    ros::Duration(0.01).sleep();
//  }

}

std::shared_ptr<fcl::CollisionObject> createCollisionObject(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr, const octomap::point3d& sensor_origin_wrt_world)
{
  // octomap octree settings
  const double resolution = 0.05;
  const double prob_hit = 0.9;
  const double prob_miss = 0.1;
  const double clamping_thres_min = 0.12;
  const double clamping_thres_max = 0.98;

  std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
  octomap_octree->setProbHit(prob_hit);
  octomap_octree->setProbMiss(prob_miss);
  octomap_octree->setClampingThresMin(clamping_thres_min);
  octomap_octree->setClampingThresMax(clamping_thres_max);

  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

#if defined(_OPENMP)
#pragma omp parallel
#endif
  {
#if defined(_OPENMP)
    auto thread_id = omp_get_thread_num();
    auto thread_num = omp_get_num_threads();
#else
    int thread_id = 0;
    int thread_num = 1;
#endif
    int start_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * thread_id;
    int end_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * (thread_id + 1);
    if (thread_id == thread_num - 1)
    {
      end_idx = pointcloud_ptr->size();
    }

    octomap::KeySet local_free_cells;
    octomap::KeySet local_occupied_cells;

    for (auto i = start_idx; i < end_idx; i++)
    {
      octomap::point3d point((*pointcloud_ptr)[i].x, (*pointcloud_ptr)[i].y, (*pointcloud_ptr)[i].z);
      octomap::KeyRay key_ray;
      if (octomap_octree->computeRayKeys(sensor_origin_3d, point, key_ray))
      {
        local_free_cells.insert(key_ray.begin(), key_ray.end());
      }

      octomap::OcTreeKey tree_key;
      if (octomap_octree->coordToKeyChecked(point, tree_key))
      {
        local_occupied_cells.insert(tree_key);
      }
    }

#if defined(_OPENMP)
#pragma omp critical
#endif
    {
      free_cells.insert(local_free_cells.begin(), local_free_cells.end());
      occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
    }
  }

  // free cells only if not occupied in this cloud
  for (auto it = free_cells.begin(); it != free_cells.end(); ++it)
  {
    if (occupied_cells.find(*it) == occupied_cells.end())
    {
      octomap_octree->updateNode(*it, false);
    }
  }

  // occupied cells
  for (auto it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
  {
    octomap_octree->updateNode(*it, true);
  }

  auto fcl_octree = std::make_shared<fcl::OcTree>(octomap_octree);
  std::shared_ptr<fcl::CollisionGeometry> fcl_geometry = fcl_octree;
  return std::make_shared<fcl::CollisionObject>(fcl_geometry);
}

double AvoidCostFunction::cost(const Eigen::VectorXd& configuration)
{
    double dist=std::numeric_limits<double>::infinity();

    //determine min time to complete straight line path from parent to config
    double dist_new = (configuration-x_near).norm();
    double time_new = (dist_new/max_vel).maxCoeff();
    double node_time_new = time_new;
    //get cost of parent
    //requires extended node data
    double c_near = 0.0;
    if (x_near.min_time == std::numeric_limits<double>::infinity()) {
        c_near = x_near.time_occupancy;
    } else {
        c_near = x_near.min_time;
    }
    // get min cost to get to new node from near parent
    double c_new = c_near + time_new;

    bool avoid_time = False;

    //some variables to store request/results of collision check between robot shapes and avoidance point cloud
    fcl::DistanceRequest<double> req(true);
    fcl::DistanceResult<double> res;
    std::vector<Eigen::Vector3d> collision_points;

    bool success = True; //bool to set false if path can not be found
    
    //convert robot to point cloud, or
    //determine which model points the robot is in collision with at configuration
    for (int i=1;i<num_links;i++) {
        fcl::CollisionObjectf* robot = new fcl::CollisionObjectf(&this->link_boxes[i], link_transforms[i]);
        fcl::collide(robot,&model.point_cloud,req,res);
        //get collision points out of res and append them to a vector
        for (pt:res....) {
            if (pt not in collision_points) {
                collision_points.push_back(pt);
            }
        }
    }
    //need to sort collision points in order of increasing x, then y, then z,
    //there will be many duplicate points since the points for each link were merged
    //and links overlap at joints
    std::sort(collision_points.begin(),collision_points.end(),sort_function);
    std::vector<Eigen::Vector4d> avoidance_intervals;
    std::vector<Eigen::Vector4d> pt_avoid_ints;
    for (int i = 0; i<collision_points.size();i++) {
        if (i<collision_points.size()-1) {
            while collision_points[i].isApprox(collision_points[i+1]) {
                i++;
            }
        }
        //look up avoidance intervals for each point robot occupies
        pt_avoid_ints = model.model_point[cart_to_model(Eigen::Vector3d collision_points[i])].avoidance_intervals;
        avoidance_intervals.insert(std::end(avoidance_intervals),std::begin(pt_avoid_ints), std::end(pt_avoid_ints));
        //merge the intervals?
        avoidance_intervals = merge_intervals(avoidance_intervals);
    }
    //determine last_pass_time for collection of avoidance intervals

    //if the min cost to get to new node from nearest parent exceeds the collective last_pass_time, then node is not viable
    if (c_new>last_pass_time){
        success = false;
        break
    }
    //if avoidance intervals, loop over avoidance intervals to determine soonest time of passage from parent to new node
    if (!avoidance_intervals.isEmpty() && success){
        bool found_avoid = false;
        double tmp_time = 0.0;
        double tmp_c_new = c_new;
        for (r=0;r<avoidance_intervals.size();r++) {
            //if time to reach new node is within an avoidance interval +/- padding then set time to reach new node to the end of the avoidance interval
            //repeat check with the time to reach the parent node when parent time is increased
            //ensuring the entire path from parent to new node is not in an avoidance interval
            if (((avoidance_intervals[r][0]-t_pad<tmp_c_new) && (tmp_c_new<avoidance_intervals[r][1]+t_pad)) || ((avoidance_intervals[r][0]-t_pad<near_time) && (near_time<avoidance_intervals[r][1]+t_pad))) {
                tmp_time = avoidance_itnervals[r][1]+node_time_new + t_pad;
                if (tmp_time>tmp_c_new) {
                    tmp_c_new = tmp_time;
                    near_time = avoidance_itnervals[r][1]+t_pad;
                }
                found_avoid = true;
            } else if (found_avoid) {
                c_new = tmp_c_new;
                found_avoid = false;
            }
        }
        if (found_avoid) {
            c_new = tmp_c_new;
        }
        c_new = std::max(c_new,tmp_c_new);
    }

    //return inf cost if not success
    if (!success) {
        c_new = std::numeric_limits<double>::infintiy();
    }

    return c_new;
}

}
