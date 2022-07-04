#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <avoidance_intervals/avoidance_model.h>

namespace avoidance_intervals{

    bool eigen_sort_rows_by_columns(Eigen::VectorXf const& r1, Eigen::VectorXf const& r2) {
        bool prev_true = true;
        for (int i=0;i<r1.size()-1;i++) {
            if (prev_true) {
                return r1(i)<r2(i);
            } else {
                break;
            }
            prev_true = prev_true && (r1(i)==r2(i));
        }
        return false;
    }

    bool sort_avoidance_intervals(Eigen::Vector3f const& r1, Eigen::Vector3f const& r2) {
        bool prev_true = true;
        for (int i=0;i<2;i++) {
            if (prev_true) {
                return r1(i)<r2(i);
            } else {
                break;
            }
            prev_true = prev_true && (r1(i)==r2(i));
        }
        return false;
    }

    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id) {
        std::sort(points.begin(),points.end(),eigen_sort_rows_by_columns);
        double min_t = points.at(0)[3];
        double max_t = min_t;
        double dt = 0.5;
        Eigen::VectorXf tmp_avoid_pt(7);
        std::vector<Eigen::VectorXf> avoid_pts;
        for (int i = 1; i<int(points.size());i++) {
            if (!points[i].head(3).isApprox(points[i-1].head(3))) {
                max_t = points[i-1][3];
                tmp_avoid_pt.head(3) = points[i-1].head(3);
                tmp_avoid_pt.tail(4) << min_t, max_t, int(max_t<end_time), id;
                avoid_pts.push_back(tmp_avoid_pt);
                min_t = points[i][3];
                max_t = min_t;
            } else if (points[i][3]-points[i-1][3]>dt) {
                max_t = points[i-1][3];
                tmp_avoid_pt.head(3) = points[i-1].head(3);
                tmp_avoid_pt.tail(4) << min_t, max_t, int(max_t<end_time), id;
                avoid_pts.push_back(tmp_avoid_pt);
                min_t = points[i][3];
            }
        }
        return avoid_pts;
    }

    void model_point::merge_intervals(void) {
        std::sort(avoidance_intervals_.begin(),avoidance_intervals_.end(),sort_avoidance_intervals);
        double min_t = avoidance_intervals_[0][0];
        double max_t = avoidance_intervals_[0][1];
        bool is_last_pass = avoidance_intervals_[0][2];
        combined_avoidance_intervals_.clear();
        Eigen::Vector3f tmp_int;
        if (avoidance_intervals_.size()>1) {
            for (int i=0;i<int(avoidance_intervals_.size());i++) {
                if ((avoidance_intervals_[i][0]<min_t) && (avoidance_intervals_[i][0]<max_t)) {
                    min_t = avoidance_intervals_[i][0];
                    is_last_pass = is_last_pass && avoidance_intervals_[i][2];
                }
                if ((avoidance_intervals_[i][1]>max_t) && (min_t < avoidance_intervals_[i][1])) {
                    max_t = avoidance_intervals_[i][1];
                }
                if (avoidance_intervals_[i][0]>max_t) {
                    tmp_int << min_t,max_t,is_last_pass;
                    combined_avoidance_intervals_.push_back(tmp_int);
                    is_last_pass = false;
                    min_t = avoidance_intervals_[i][0];
                    max_t = avoidance_intervals_[i][1];
                    is_last_pass = avoidance_intervals_[i][2];
                }
            }
        }
    }

    model::model(const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) 
    {
        Eigen::VectorXd range_down = (lower_bound.array()/grid_spacing).floor().array()*grid_spacing;
        Eigen::VectorXd range_up = (upper_bound.array()/grid_spacing).ceil().array()*grid_spacing;
        num_bins = ((range_up-range_down).array()/grid_spacing).cast <int> () + 1;
    }

    int model::cart_to_model(Eigen::Vector3f pt) {
        return pt[2]*num_bins[1]*num_bins[0]+pt[1]*num_bins[0]+pt[0];
    }

    void model::generate_model_cloud(std::vector<Eigen::VectorXf> avoid_pts) {
        model_point tmp_point;
        Eigen::Vector3f tmp_interval;
        int idx = 0;
        for (int i=0;i<int(avoid_pts.size());i++){
            tmp_interval << avoid_pts[i][3], avoid_pts[i][4], avoid_pts[i][6];
            idx = cart_to_model(avoid_pts[i].head(3));
            model_points[idx].position = avoid_pts[i].head(3);
            model_points[idx].avoidance_intervals_.push_back(tmp_interval);
            if ((!avoid_pts[i][5]) && (avoid_pts[i][3]<model_points[idx].last_pass_time_)) {
                model_points[idx].last_pass_time_ = avoid_pts[i][3];
            }
            model_points[idx].allows_passage_ = model_points[idx].allows_passage_ and avoid_pts[i][5];
        }
        octomap::OcTree* tree = new octomap::OcTree(0.05);
        for (model_point pt:model_points) {
            tree->updateNode(octomap::point3d(pt.position[0],pt.position[1],pt.position[2]), true);
        }
        fcl::OcTree<float>* tree_ptr = new fcl::OcTree<float>(std::shared_ptr<octomap::OcTree>(tree));
        std::shared_ptr<fcl::CollisionGeometry<float>> pt_cld_ptr(tree_ptr);
        point_cloud_ = new fcl::CollisionObjectf(pt_cld_ptr);
        // point_cloud_ = std::make_shared<fcl::CollisionObject<double>>(pt_cld_ptr);
    }
    void merge_intervals(std::vector<Eigen::Vector3f> intervals, std::vector<Eigen::Vector3f> &combined_intervals, float &last_pass_time ) {
        std::sort(intervals.begin(),intervals.end(),sort_avoidance_intervals);
        double min_t = intervals[0][0];
        double max_t = intervals[0][1];
        bool is_last_pass = intervals[0][3];
        combined_intervals.clear();
        last_pass_time = std::numeric_limits<float>::infinity();
        Eigen::Vector3f tmp_int;
        if (intervals.size()>1) {
            for (int i=0;i<int(intervals.size());i++) {
                if ((intervals[i][0]<min_t) && (intervals[i][0]<max_t)) {
                    min_t = intervals[i][0];
                    is_last_pass = is_last_pass && intervals[i][2];
                    if (intervals[i][2]) {
                        last_pass_time = std::min(last_pass_time, intervals[i][0]);
                    }
                }
                if ((intervals[i][1]>max_t) && (min_t < intervals[i][1])) {
                    max_t = intervals[i][1];
                }
                if (intervals[i][0]>max_t) {
                    tmp_int << min_t,max_t,is_last_pass;
                    combined_intervals.push_back(tmp_int);
                    is_last_pass = false;
                    min_t = intervals[i][0];
                    max_t = intervals[i][1];
                    is_last_pass = intervals[i][2];
                }
            }
        }
    }

    Eigen::MatrixXf generate_cylinder_pts(double radius, double length, double grid_size) {
        double grid_size_=0.7*grid_size;
        int z_steps = ceil(length/grid_size_);
        double r_grid = radius/grid_size_;
        double r_grid2 = r_grid*r_grid;
        int r_steps = ceil(r_grid);
        int vol = 4*ceil((r_steps+1)*(r_steps+1)*(z_steps+1));
        Eigen::MatrixXf pts(3,vol);
        int pt_idx = 0;
        for (int z=0;z<=z_steps;z++) {
            for (int x=0;x<=r_steps;x++) {
                int circle_val = ceil(sqrt(std::max(r_grid2-x*x,0.0)));
                for (int y=0;y<=circle_val;y++) {
                    pts.col(pt_idx) << x,y,z;
                    pt_idx++;
                    pts.col(pt_idx) << -x,y,z;
                    pt_idx++;
                    pts.col(pt_idx) << x,-y,z;
                    pt_idx++;
                    pts.col(pt_idx) << -x,-y,z;
                    pt_idx++;
                }
            }
        }
        // pts.col(pt_idx) << 0,0,0;
        // pt_idx++;
        // pts.col(pt_idx) << 0,0,z_steps;
        // pt_idx++;
        pts.conservativeResize(3,pt_idx);
        pts=pts*grid_size_;
        return pts;
    }

    void downsample_points(Eigen::MatrixXf& pts,double grid_size) {
        pts=pts*(1/grid_size);
        pts = pts.array().round().matrix();
        pts=pts*grid_size;
    }

    std::vector<Eigen::Vector4f> remove_duplicates_add_time(std::vector<Eigen::MatrixXf> all_pts, double t,std::vector<double> other_times,double t_step) {
        std::vector<Eigen::Vector3f> pts_vec;
        for (Eigen::MatrixXf pts: all_pts) {
            for (int i=0;i<pts.cols();i++) {
                pts_vec.push_back(pts.col(i));
            }
        }
        std::sort(pts_vec.begin(),pts_vec.end(),eigen_sort_rows_by_columns);
        std::vector<Eigen::Vector4f> pts_t_vec;
        Eigen::Vector4f t_pt;
        for (int i=0;i<pts_vec.size()-1;i++) {
            if (pts_vec[i].isApprox(pts_vec[i+1])) {
                continue;
            }
            int min_t_step = floor(t/t_step);
            int max_t_step = ceil(t/t_step);
            for (double other_t:other_times) {
                if (other_t<t) {
                    min_t_step = floor(other_t/t_step);
                } else {
                    max_t_step = ceil(other_t/t_step);
                }
            }                
            for (int ti=min_t_step;ti<=max_t_step;ti++) {
                t_pt << pts_vec[i][0],pts_vec[i][1],pts_vec[i][2],t_step*ti;
                std::cout<<t_pt.transpose()<<std::endl;
                pts_t_vec.push_back(t_pt);
            }
        }
        return pts_t_vec;
    }

    skeleton::skeleton(ros::NodeHandle nh, const std::string topic_name,double grid_size, double t_step) {
        sub_skeletons = nh.subscribe(topic_name,1,&skeleton::callback, this);
        grid_size_ = grid_size;
        t_step_ = t_step;
    }

    void skeleton::forward_kinematics(std::vector<float> pose_elements,std::vector<double> t_steps) {
        ROS_INFO_STREAM("forward kinematics");
        Eigen::Vector3f pelvis_loc = {pose_elements[1],pose_elements[2],pose_elements[3]};

        Eigen::Quaternionf z_axis_quat(0,0,0,1);
        std::vector<Eigen::Quaternionf> quats;
        Eigen::Quaternionf q;
        for (int i=0;i<7;i++){
            q = Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]);
            quats.push_back(q);
            ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
        }
        Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
        Eigen::Vector3f spine_top = pelvis_loc+link_lengths_[0]*z_spine.vec();
        Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
        Eigen::Vector3f head = spine_top+link_lengths_[1]*z_neck.vec();
        Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
        Eigen::Vector3f l_shoulder = spine_top-0.5*link_lengths_[2]*z_shoulders.vec();
        Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
        Eigen::Vector3f e1 = l_shoulder+link_lengths_[3]*z_e1.vec();
        Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
        Eigen::Vector3f w1 = e1+(link_lengths_[4]+0.1)*z_w1.vec();
        Eigen::Vector3f r_shoulder = spine_top+0.5*link_lengths_[2]*z_shoulders.vec();
        Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
        Eigen::Vector3f e2 = r_shoulder+link_lengths_[5]*z_e2.vec();
        Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
        Eigen::Vector3f w2 = e2+(link_lengths_[6]+0.1)*z_w2.vec();

        std::vector<Eigen::Matrix3f> rotations;
        std::vector<Eigen::MatrixXf> all_pts;
        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f r_y=Eigen::Matrix3f::Identity();
        r_y= Eigen::Quaternionf(0,0,1,0);

        for (int i=0;i<7;i++){
            r = Eigen::Quaternionf(quats[i]);
            rotations.push_back(r);
            ROS_INFO_STREAM("rotation \n"<<r);
        }

        Eigen::MatrixXf oversampled_pts = (rotations[0]*raw_limb_points[0]).colwise() + pelvis_loc;
        oversampled_pts=r_y*oversampled_pts;
        ROS_INFO_STREAM("pose torso \n"<<oversampled_pts);
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose torso \n"<<oversampled_pts);

        // Eigen::Vector3f spine_top = pelvis_loc+rotations[0]*Eigen::Vector3f(0,0,link_lengths_[0]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[1]*raw_limb_points[1]).colwise() + spine_top;
        oversampled_pts=r_y*oversampled_pts;
        ROS_INFO_STREAM("pose neck \n"<<oversampled_pts);
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose neck \n"<<oversampled_pts);

        // Eigen::Vector3f shoulder_vec=rotations[2]*Eigen::Vector3f(0,0,0.5*link_lengths_[2]);
        
        // Eigen::Vector3f origin = spine_top-shoulder_vec;
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[3]*raw_limb_points[2]).colwise() + l_shoulder;
        oversampled_pts=r_y*oversampled_pts;
        ROS_INFO_STREAM("pose l up \n"<<oversampled_pts);
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose l up \n"<<oversampled_pts);

        // origin = origin+rotations[3]*Eigen::Vector3f(0,0,link_lengths_[3]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[4]*raw_limb_points[3]).colwise()  + e1;
        oversampled_pts=r_y*oversampled_pts;
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose l fore \n"<<oversampled_pts);
                
        // origin = spine_top+shoulder_vec;
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[5]*raw_limb_points[4]).colwise() + r_shoulder;
        oversampled_pts=r_y*oversampled_pts;
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose r up \n"<<oversampled_pts);

        // origin = origin+rotations[5]*Eigen::Vector3f(0,0,link_lengths_[5]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[6]*raw_limb_points[5]).colwise() + e2;
        oversampled_pts=r_y*oversampled_pts;
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        ROS_INFO_STREAM("pose r fore \n"<<oversampled_pts);
        ROS_INFO_STREAM("end");
        std::vector<Eigen::Vector4f> reduced_pts = remove_duplicates_add_time(all_pts,pose_elements[0],t_steps,t_step_);
        transformed_points.insert(transformed_points.end(),reduced_pts.begin(),reduced_pts.end());
        
    }

    void skeleton::callback(const human_motion_prediction::human_pose::ConstPtr& msg) {
        link_lengths_ = std::vector<float>(msg->link_lengths.data);
        link_radii_ = std::vector<float>(msg->link_radii.data);

        if (raw_limb_points.empty()){
            for (int i=0;i<link_lengths_.size();i++) {
                std::cout<<link_lengths_[i]<<", "<<link_radii_[i]<<", ";
                if (i==2) continue;
                Eigen::MatrixXf tmp_pts = generate_cylinder_pts(link_radii_[i],link_lengths_[i],grid_size_);
                raw_limb_points.push_back(tmp_pts);
            }
            std::cout<<std::endl;
        }
        // std::cout<<"press enter\n";
        // std::cin.ignore();
        //this float32multiarray msg was structured with 2 dimensions
        //dim[0] is the number of quat elements per pose
        //dim[1] is the number of time steps predicted into the future
        //get the dimensions from the message
        // float dstride0 = msg->prediction.layout.dim[0].stride;
        // float dstride1 = msg->prediction.layout.dim[1].stride;
        float h = msg->prediction.layout.dim[0].size;
        float w = msg->prediction.layout.dim[1].size;
        //cast the msg data to a vector of floats
        std::vector<float> data = msg->prediction.data;
        //loop over timesteps in msg
        transformed_points.clear();
        for (int i=0;i<w;i++){
            std::vector<double> t_steps;

            //compute the human points via forward kinematics
            if (w>0) {
                if (i>0) {
                    t_steps.push_back(data[i*h]);
                }
                if (i<w-1) {
                    t_steps.push_back(data[(i+1)*h]);
                }
                forward_kinematics(std::vector<float>(data.begin()+i*h,data.begin()+(i+1)*h-1),t_steps);
            }
        }
        ROS_INFO_STREAM("t2");
        std::lock_guard<std::mutex> l(_mtx);
        ready_points = transformed_points;
        end_time_ = data[(w-1)*h];
        std::vector<Eigen::VectorXf> avoid_pts = determine_intervals(transformed_points,end_time_,1);
        ready = true;
        
    }

    void model::display_markers(void) {

    }

}