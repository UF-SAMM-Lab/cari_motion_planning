#include <ros/ros.h>
#include <avoidance_intervals/avoidance_model.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_model");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    avoidance_intervals::skeleton test_skeleton(nh,"predicted_skeleton",0.05,1.0);
    Eigen::Vector3d model_up_bound={2.0,2.0,2.0};
    Eigen::Vector3d model_lower_bound={-2.0,-2.0,0.0};
    avoidance_intervals::model test_model(model_lower_bound,model_up_bound);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "human_markers", 0 );
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
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();//ros::Time::now();
            marker.ns = "my_namespace";
            marker.id = 0;
            marker.lifetime = ros::Duration(0.2);
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 1;
            marker.pose.position.y = 1;
            marker.pose.position.z = 1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
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
            p.y = 1;
            p.z = 0;
            marker.points.push_back(p);
            p.x = 0;
            p.y = 0;
            p.z = 1;
            marker.points.push_back(p);
            for (Eigen::Vector4f pt:pts) {
                if (pt[3]==t*test_skeleton.t_step_) {
                    p.x = pt[0];
                    p.y = pt[1];
                    p.z = pt[2];
                    marker.points.push_back(p);
                }
            }
            if (marker.points.size()>0) vis_pub.publish(marker);
            ROS_INFO_STREAM("skeleton markers published");
            // std::cin.ignore();
            ros::Duration(0.2).sleep();
        }
        test_skeleton.ready = false;
        ros::Duration(0.2).sleep();
    }
    return 0;
}