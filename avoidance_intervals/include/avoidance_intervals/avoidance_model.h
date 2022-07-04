#pragma once
#pragma GCC diagnostic ignored "-Wsign-compare"

#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <fcl/narrowphase/detail/gjk_solver_indep.h>
#include <fcl/narrowphase/detail/gjk_solver_libccd.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/math/bv/utility.h>
#include <fcl/geometry/collision_geometry-inl.h>
#include <fcl/geometry/bvh/detail/BV_fitter.h>
#include <fcl/geometry/octree/octree.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>
#include <human_motion_prediction/human_pose.h>
#include <math.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <mutex>

namespace avoidance_intervals{

    bool eigen_sort_rows_by_columns(Eigen::VectorXf const& r1, Eigen::VectorXf const& r2);

    bool sort_avoidance_intervals(Eigen::Vector3f const& r1, Eigen::Vector3f const& r2);

    void merge_intervals(std::vector<Eigen::Vector3f> intervals, std::vector<Eigen::Vector3f> &combined_intervals, float &last_pass_time );

    Eigen::MatrixXf generate_cylinder_pts(double radius, double length, double grid_size);

    void downsample_points(Eigen::MatrixXf& pts,double grid_size);

    std::vector<Eigen::Vector4f> remove_duplicates_add_time(std::vector<Eigen::MatrixXf> all_pts, double t,std::vector<double> other_times,double t_step);
    
    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id);

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
        double grid_spacing;
        Eigen::VectorXi num_bins;
        std::vector<model_point> model_points;
        fcl::CollisionObjectf* point_cloud_;
        // std::shared_ptr<fcl::CollisionObject<double>> point_cloud_;
        int cart_to_model(Eigen::Vector3f pt);
        void generate_model_cloud(std::vector<Eigen::VectorXf> avoid_pts);
        void display_markers(void);
        model(const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound);
        ~model() {};
    };
    typedef std::shared_ptr<model> modelPtr;

    class skeleton {
        public:
            mutable std::mutex _mtx;
            std::vector<Eigen::MatrixXf> raw_limb_points;
            std::vector<Eigen::Vector4f> transformed_points;
            std::vector<Eigen::Vector4f> ready_points;
            skeleton(ros::NodeHandle nh, const std::string topic_name,double grid_size, double t_step);
            void forward_kinematics(std::vector<float> pose_elements,std::vector<double> t_steps);
            void callback(const human_motion_prediction::human_pose::ConstPtr& msg);
            double end_time_;
            double t_step_;
            bool ready =  false;
        protected:
            double grid_size_;
            ros::Subscriber sub_skeletons;
        private:
            std::vector<float> link_lengths_;
            std::vector<float> link_radii_;
    };
}