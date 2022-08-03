#pragma once
#pragma GCC diagnostic ignored "-Wsign-compare"

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>
#include <human_motion_prediction/human_pose.h>
#include <math.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <mutex>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <string>
#include <sstream>
#include <exception>
#include <ros/package.h>
#include <thread>
#include <algorithm>
#include <fstream>

namespace avoidance_intervals{

    bool eigen_sort_rows_by_columns(Eigen::VectorXf const& r1, Eigen::VectorXf const& r2);

    bool sort_avoidance_intervals(Eigen::Vector3f const& r1, Eigen::Vector3f const& r2);

    void merge_intervals(std::vector<Eigen::Vector3f> intervals, std::vector<Eigen::Vector3f> &combined_intervals, float &last_pass_time );

    Eigen::MatrixXf generate_cylinder_pts(double radius, double length, double grid_size);

    void downsample_points(Eigen::MatrixXf& pts,double grid_size);

    std::vector<Eigen::Vector4f> remove_duplicates_add_time(std::vector<Eigen::MatrixXf> all_pts, double t,std::vector<double> other_times,double t_step);
    
    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id);
    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id, visualization_msgs::Marker &marker);

    void display_markers(std::vector<Eigen::Vector4f> pts,int t,double t_step, visualization_msgs::Marker &marker);

    void display_markers(std::vector<Eigen::Vector4f> pts,visualization_msgs::Marker &marker);
    void display_markers(std::vector<Eigen::MatrixXf> pts,visualization_msgs::Marker &marker);
    void display_markers(std::vector<Eigen::VectorXf> pts,visualization_msgs::Marker &marker);

    void read_human_task(int task_num);

    class model_point
    {
    public:
    std::vector<Eigen::Vector3f> avoidance_intervals_;
    std::vector<Eigen::Vector3f> combined_avoidance_intervals_;
    Eigen::Vector3f position;
    bool allows_passage_;
    float last_pass_time_;
    void merge_intervals(void);
    };

    class model {
        public:
        Eigen::MatrixXd ranges;
        Eigen::VectorXf range_down;
        Eigen::VectorXf range_up;
        float grid_spacing;
        Eigen::Vector3i num_bins;
        std::vector<model_point> model_points;
        std::vector<int>model_pt_idx;
        int cart_to_model(Eigen::Vector3f pt);
        void generate_model_cloud(std::vector<Eigen::VectorXf> avoid_pts);
        void generate_model_cloud(void);
        void clear_model_cloud(void);
        model(const Eigen::Vector3f& lower_bound, const Eigen::Vector3f& upper_bound, float grid_space, int num_threads, ros::NodeHandle nh);
        ~model() {};
        void displayAllRequest(void);
        Eigen::Matrix3Xf avoid_cart_pts;
        std::mutex ext_mutex;
        private:
        std::vector<std::thread> threads;
        std::vector<Eigen::VectorXf> avoid_pts_;
        std::vector<Eigen::VectorXf> avoid_pts_msg;
        std::vector<float> msg_data;
        std::mutex mtx;
        std::mutex msg_mtx;
        std::mutex pts_mtx;
        void generate_thread(int start, int end, int thread_num);
        void displayAllRequestCallback(const std_msgs::Bool::ConstPtr& msg);

        void humanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void displayRequestCallback(const std_msgs::Float32::ConstPtr& msg);
        void callbackThread(int start, int end, int h);
        void subscribeTopic();
        ros::Subscriber poses_sub;
        ros::Subscriber disp_sub;
        ros::Subscriber disp_all_sub;
        ros::Publisher disp_pub;
        protected:
        int num_threads_;
        ros::NodeHandle nh_;
    };
    typedef std::shared_ptr<model> modelPtr;

    class skeleton {
        public:
            mutable std::mutex _mtx;
            mutable std::mutex _mtx_read;
            std::vector<Eigen::MatrixXf> raw_limb_points;
            std::vector<Eigen::Vector4f> transformed_points;
            std::vector<Eigen::Vector4f> ready_points;
            skeleton(ros::NodeHandle nh, const std::string topic_name,double grid_size, double t_step);
            void forward_kinematics(std::vector<float> pose_elements,std::vector<double> t_steps);//,std_msgs::Float32MultiArray prediction);
            void callback(const human_motion_prediction::human_pose::ConstPtr& msg);
            void publish_pts(std::vector<Eigen::VectorXf> pts);
            void read_thread(std::vector<std::string> in_lines, std::string prev_line, std::string next_line,int thread_num);
            std::vector<Eigen::VectorXf> read_human_task(int task_num, Eigen::Isometry3f transform);
            double end_time_;
            double t_step_;
            bool ready =  false;
            std::vector<float> link_lengths_;
            std::vector<float> link_radii_;
        private:
            Eigen::Isometry3f transform_to_world;
        protected:
            int num_threads_;
            double grid_size_;
            ros::Subscriber sub_skeletons;
            ros::NodeHandle nh_;
            ros::Publisher pts_pub_;
            ros::Publisher vis_pub_;
    };
}