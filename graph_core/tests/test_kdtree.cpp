#include <ros/ros.h>
#include <graph_core/graph/kdtree.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_kdtree");
  ros::NodeHandle nh;

  size_t trials=100;
  size_t repetions=10;
  double radius=0.5;
  Eigen::VectorXd q(6);

  for (size_t magnitude=2;magnitude<=5;magnitude++)
  {
    double populating_kdtree_time=0.0;
    double populating_vector_time=0.0;
    double nn_kdtree_time=0.0;
    double nn_vector_time=0.0;
    double near_kdtree_time=0.0;
    double near_vector_time=0.0;

    size_t n_nodes=std::pow(10,magnitude);

    ROS_INFO("\n\nBuilding tree with %zu nodes, cardinality = %zu",n_nodes,q.size());

    for (size_t rep=0;rep<repetions;rep++)
    {
      pathplan::KdTree kdtree;
      pathplan::NodePtr n;
      std::vector<pathplan::NodePtr> vector_nodes;

      for (size_t idx=0;idx<n_nodes;idx++)
      {
        q.setRandom();
        n=std::make_shared<pathplan::Node>(q);
        ros::Time tnn=ros::Time::now();
        kdtree.insert(n);
        populating_kdtree_time+=(ros::Time::now()-tnn).toSec();
        ROS_DEBUG_STREAM("inserting node = " << n->getConfiguration().transpose());

        tnn=ros::Time::now();
        vector_nodes.push_back(n);
        populating_vector_time+=(ros::Time::now()-tnn).toSec();
      }

      for (size_t idx=0;idx<trials;idx++)
      {
        if (not ros::ok())
          return 0;
        q.setRandom();

        double best_distance;
        ros::Time tnn=ros::Time::now();
        kdtree.nearestNeighbor(q,n,best_distance);
        nn_kdtree_time+=(ros::Time::now()-tnn).toSec();
        ROS_DEBUG_STREAM("nearestNeighbor node = " << n->getConfiguration().transpose()<< " w.r.t q = "<<q.transpose());

        tnn=ros::Time::now();
        double vector_dist=std::numeric_limits<double>::infinity();
        pathplan::NodePtr vector_nn;
        for (pathplan::NodePtr& n: vector_nodes)
        {
          double dist=(n->getConfiguration()-q).norm();
          if (dist<vector_dist)
          {
            vector_dist=dist;
            vector_nn=n;
          }
        }
        nn_vector_time+=(ros::Time::now()-tnn).toSec();

        if ((vector_nn->getConfiguration()-n->getConfiguration()).norm()>1e-6)
        {
          ROS_INFO("some errors");
        }
      }




      ROS_DEBUG_STREAM("Nodes in ball of radius = "<< radius );

      for (size_t idx=0;idx<trials;idx++)
      {
        if (not ros::ok())
          return 0;

        q.setRandom();
        ros::Time tnn=ros::Time::now();
        std::map<double, pathplan::NodePtr> nodes=kdtree.near(q,radius);
        near_kdtree_time+=(ros::Time::now()-tnn).toSec();
        for (const std::pair<double, pathplan::NodePtr>& p: nodes)
        {
          ROS_DEBUG_STREAM("- node = " << p.second->getConfiguration().transpose()<< ", distance = " << p.first);
        }

        tnn=ros::Time::now();
        std::map<double, pathplan::NodePtr> nodes2;
        for (pathplan::NodePtr& n: vector_nodes)
        {
          double dist=(n->getConfiguration()-q).norm();
          if (dist<radius)
          {
            nodes2.insert(std::pair<double,pathplan::NodePtr>(dist,n));
          }
        }
        near_vector_time+=(ros::Time::now()-tnn).toSec();
      }


      //    size_t k=4;
      //    ROS_DEBUG_STREAM(k<<"-Nearest Neighbors around q = "<< q.transpose());
      //    std::map<double, pathplan::NodePtr> knn=kdtree.kNearestNeighbors(q,k);
      //    for (const std::pair<double, pathplan::NodePtr>& p: knn)
      //    {
      //      ROS_DEBUG_STREAM("- node = " << p.second->getConfiguration().transpose() << ", distance = " << p.first);
      //    }

      //    ROS_DEBUG_STREAM(k<<"-Nearest Neighbors around q = "<< q.transpose());
      //    kdtree.deleteNode(knn.begin()->second);
      //    knn=kdtree.kNearestNeighbors(q,k);
      //    for (const std::pair<double, pathplan::NodePtr>& p: knn)
      //    {
      //      ROS_DEBUG_STREAM("- node = " << p.second->getConfiguration().transpose() << ", distance = " << p.first);
      //    }

    }
    ROS_INFO("Building  kdtree= %f ms, vector = %f ms, ratio = %f ",populating_kdtree_time*1e3/repetions,populating_vector_time*1e3/repetions,populating_kdtree_time/populating_vector_time);
    ROS_INFO("NN        kdtree= %f ms, vector = %f ms, ratio = %f ",nn_kdtree_time*1e3/trials/repetions,nn_vector_time*1e3/trials/repetions,nn_kdtree_time/nn_vector_time);
    ROS_INFO("Near      kdtree= %f ms, vector = %f ms, ratio = %f ",near_kdtree_time*1e3/trials/repetions,near_vector_time*1e3/trials/repetions,near_kdtree_time/near_vector_time);

  }
  return 0;
}
