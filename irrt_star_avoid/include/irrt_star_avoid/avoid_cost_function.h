#pragma once

#include <rosdyn_core/primitives.h>
#include <graph_core/goal_cost_function.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

namespace pathplan
{

class AvoidCostFunction: public GoalCostFunction
{
protected:

  ros::NodeHandle nh_;
  rosdyn::ChainPtr chain_;

  double min_distance_=0.2;
  double max_distance_=1.0;
  double inv_delta_distance_;
  double max_penalty_=2.0;
  bool plot=false;
  std::vector<std::string> links_;

  Eigen::Matrix<double,3,-1> points_;

  ros::Publisher marker_pub_;
  int marker_id_;
public:
AvoidanceCostFunction(const ros::NodeHandle &nh);
void cleanPoints();
void addPoint(const Eigen::Vector3d &point);
void publishPoints();
virtual double cost(const Eigen::VectorXd& q);
};
typedef std::shared_ptr<AvoidCostFunction> AvoidCostFunctionPtr;

}