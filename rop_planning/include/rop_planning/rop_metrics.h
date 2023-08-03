#include <Eigen/Core>
#include <ros/ros.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <rosdyn_core/primitives.h>


namespace pathplan
{
    class ROPMetrics
    {   
        public:
        Eigen::Tensor<double,3> m_occupancy;
        Eigen::Vector3d workspace_lb={-1,-1,0.5};
        Eigen::Vector3d workspace_ub={1,1,2.5};
        std::vector<int> m_npnt;
        rosdyn::ChainPtr chain;
        std::vector<std::string> link_names;
        std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> m_transformations;
        std::map<std::string,std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >> m_test_points;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_points;
        std::vector<std::string> links_to_check;
        double step_;
        double m_resolution=0.05;
        ros::Subscriber sub_opvs;
    };

}