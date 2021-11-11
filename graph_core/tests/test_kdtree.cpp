#include <ros/ros.h>
#include <graph_core/graph/kdtree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_kdtree");
  ros::NodeHandle nh;

  pathplan::KdTree kdtree;
  ROS_INFO("Population tree");
  Eigen::VectorXd q(2);
  pathplan::NodePtr n;

  q<<51,72;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);


  q<<25,40;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<70,70;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<10,30;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<35,90;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<55,1;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<60,80;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);


  q<<1,10;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<50,50;
  n=std::make_shared<pathplan::Node>(q);
  ROS_INFO_STREAM("inserting node = " << n->getConfiguration().transpose());
  kdtree.insert(n);

  q<<52,52;
  double best_distance;
  kdtree.nearestNeighbor(q,n,best_distance);
  ROS_INFO_STREAM("nearestNeighbor node = " << n->getConfiguration().transpose()<< " w.r.t q = "<<q.transpose());


  double radius=30;
  ROS_INFO_STREAM("Nodes in ball of radius = "<<radius <<" around q = "<< q.transpose());
  std::map<double, pathplan::NodePtr> nodes=kdtree.near(q,radius);
  for (const std::pair<double, pathplan::NodePtr>& p: nodes)
  {
    ROS_INFO_STREAM("- node = " << p.second->getConfiguration().transpose()<< ", distance = " << p.first);
  }


  size_t k=4;
  ROS_INFO_STREAM(k<<"-Nearest Neighbors around q = "<< q.transpose());
  std::map<double, pathplan::NodePtr> knn=kdtree.kNearestNeighbors(q,k);
  for (const std::pair<double, pathplan::NodePtr>& p: knn)
  {
    ROS_INFO_STREAM("- node = " << p.second->getConfiguration().transpose() << ", distance = " << p.first);
  }

  ROS_INFO_STREAM(k<<"-Nearest Neighbors around q = "<< q.transpose());
  kdtree.deleteNode(knn.begin()->second);
  knn=kdtree.kNearestNeighbors(q,k);
  for (const std::pair<double, pathplan::NodePtr>& p: knn)
  {
    ROS_INFO_STREAM("- node = " << p.second->getConfiguration().transpose() << ", distance = " << p.first);
  }
  return 0;
}
