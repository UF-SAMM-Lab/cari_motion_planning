#include <ros/ros.h>
#include <avoidance_intervals/avoidance_model.h>

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_model");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    avoidance_intervals::skeleton test_skeleton(nh,"predicted_skeleton",0.05,0.1);
    Eigen::Vector3f model_up_bound={2.0,2.0,2.0};
    Eigen::Vector3f model_lower_bound={-2.0,-2.0,0.0};
    avoidance_intervals::model test_model(model_lower_bound,model_up_bound,0.05,4,nh);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "human_markers", 0 );
    if (0) {
        while (!test_skeleton.ready) {
            ros::Duration(1.0).sleep();
            ROS_INFO_STREAM("waiting for skeleton ready");
        }
        ROS_INFO_STREAM("skeleton ready");
        while (ros::ok()) {
            while (!test_skeleton.ready) {
                ros::Duration(1.0).sleep();
                ROS_INFO_STREAM("waiting for skeleton ready inside");
            }
            visualization_msgs::Marker marker;
            test_skeleton._mtx.lock();
            std::vector<Eigen::Vector4f> pts = test_skeleton.ready_points;
            test_skeleton._mtx.unlock();
            for (int t=0;t<test_skeleton.end_time_/test_skeleton.t_step_;t++) {
                avoidance_intervals::display_markers(pts,t,test_skeleton.t_step_,marker);
        
                if (marker.points.size()>0) vis_pub.publish(marker);
                ROS_INFO_STREAM("skeleton markers published");
                // std::cin.ignore();
                ros::Duration(0.2).sleep();
            }
            test_skeleton.ready = false;
            ros::Duration(0.2).sleep();
        }
    } else {
        test_skeleton.link_lengths_ = {0.5,0.2,0.3,0.25,0.25,0.25,0.25};
        test_skeleton.link_radii_ = {0.12,0.05,0.1,0.04,0.03,0.04,0.03};
        std::vector<Eigen::VectorXf> avoid_pts = test_skeleton.read_human_task(0,Eigen::Isometry3f::Identity());
        visualization_msgs::Marker marker;
        test_skeleton._mtx.lock();
        std::vector<Eigen::Vector4f> pts = test_skeleton.ready_points;
        test_skeleton._mtx.unlock();
        for (int t=0;t<test_skeleton.end_time_/test_skeleton.t_step_;t++) {
            avoidance_intervals::display_markers(pts,t,test_skeleton.t_step_,marker);
    
            if (marker.points.size()>0) {
                vis_pub.publish(marker);
                ROS_INFO_STREAM("skeleton markers published");
            }
            std::cin.ignore();
            ros::Duration(0.2).sleep();
        }
    }
    return 0;
}