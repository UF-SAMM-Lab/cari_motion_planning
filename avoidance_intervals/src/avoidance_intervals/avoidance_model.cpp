#include <avoidance_intervals/avoidance_model.h>

namespace avoidance_intervals{

    bool eigen_sort_rows_by_columns(Eigen::VectorXf const& r1, Eigen::VectorXf const& r2) {
        Eigen::VectorXi r1i = (r1*1000).cast <int> ();
        Eigen::VectorXi r2i = (r2*1000).cast <int> ();
        bool prev_true = true;
        for (int i=0;i<r1.size();i++) {//-1
            if (prev_true) {
                if (r1i(i)<r2i(i)) return true;
            } else {
                break;
            }
            prev_true = prev_true && (r1i(i)==r2i(i));
        }
        return false;
    }

    bool sort_avoidance_intervals(Eigen::Vector3f const& r1, Eigen::Vector3f const& r2) {
        bool prev_true = true;
        for (int i=0;i<2;i++) {
            if (prev_true) {
                if (r1(i)<r2(i)) return true;
            } else {
                break;
            }
            prev_true = prev_true && (r1(i)==r2(i));
        }
        return false;
    }

    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id){
        visualization_msgs::Marker marker;
        return determine_intervals(points,end_time,id,marker);
    }
    std::vector<Eigen::VectorXf> determine_intervals(std::vector<Eigen::Vector4f> points, float end_time, int id, visualization_msgs::Marker &marker) {
        // ROS_INFO_STREAM("end time:"<<end_time);
        std::sort(points.begin(),points.end(),eigen_sort_rows_by_columns);
        // display_markers(points,marker);
        double min_t = points.at(0)[3];
        double max_t = min_t;
        double dt = 0.2;
        Eigen::VectorXf tmp_avoid_pt(7);
        std::vector<Eigen::VectorXf> avoid_pts;
        for (int i = 1; i<int(points.size());i++) {
            // std::cout<<points[i].transpose()<<std::endl;
            //probably broken here
            if ((points[i][0]!=points[i-1][0])||(points[i][1]!=points[i-1][1])||(points[i][2]!=points[i-1][2]))  {
                max_t = points[i-1][3];
                tmp_avoid_pt.head(3) = points[i-1].head(3);
                tmp_avoid_pt.tail(4) << min_t, max_t, int(max_t>=end_time), id;
                avoid_pts.push_back(tmp_avoid_pt);
                // ROS_INFO_STREAM("new point"<<points[i-1].transpose()<<", "<<points[i].transpose()<<", "<<tmp_avoid_pt.transpose());
                min_t = points[i][3];
                max_t = min_t;
            } else if (points[i][3]-points[i-1][3]>dt) {
                max_t = points[i-1][3];
                tmp_avoid_pt.head(3) = points[i-1].head(3);
                tmp_avoid_pt.tail(4) << min_t, max_t, int(max_t>=end_time), id;
                avoid_pts.push_back(tmp_avoid_pt);
                // ROS_INFO_STREAM("same point different time"<<points[i-1].transpose()<<", "<<points[i].transpose()<<", "<<tmp_avoid_pt.transpose());
                min_t = points[i][3];
            }// else {
            //     ROS_INFO_STREAM("same point "<<points[i-1].transpose()<<", "<<points[i].transpose());
            // }
        }
        return avoid_pts;
    }

    void model_point::merge_intervals(void) {

        if (avoidance_intervals_.empty()) return;
        std::sort(avoidance_intervals_.begin(),avoidance_intervals_.end(),sort_avoidance_intervals);
        double min_t = avoidance_intervals_[0][0];
        double max_t = avoidance_intervals_[0][1];
        bool is_last_pass = avoidance_intervals_[0][2];
        combined_avoidance_intervals_.clear();
        Eigen::Vector3f tmp_int;

        if (avoidance_intervals_.size()>1) {
            for (int i=0;i<int(avoidance_intervals_.size());i++) {
                // std::cout<<"min t"<<min_t<<", max t:"<<max_t<<"intervals:"<<intervals[i].transpose()<<std::endl;
                if ((avoidance_intervals_[i][0]<min_t) && (avoidance_intervals_[i][0]<max_t)) {
                    min_t = avoidance_intervals_[i][0];
                    is_last_pass = is_last_pass && avoidance_intervals_[i][2];
                }
                if ((avoidance_intervals_[i][1]>max_t) && (avoidance_intervals_[i][0]<max_t)) {
                    max_t = avoidance_intervals_[i][1];
                }
                if (avoidance_intervals_[i][0]>max_t) {
                    tmp_int << min_t,max_t,is_last_pass;
                    // std::cout<<"tmp_int:"<<tmp_int.transpose()<<std::endl;
                    combined_avoidance_intervals_.push_back(tmp_int);
                    is_last_pass = false;
                    min_t = avoidance_intervals_[i][0];
                    max_t = avoidance_intervals_[i][1];
                    is_last_pass = avoidance_intervals_[i][2];
                }
            }
        }

        tmp_int << min_t,max_t,is_last_pass;
        // std::cout<<"tmp_int:"<<tmp_int.transpose()<<std::endl;
        combined_avoidance_intervals_.push_back(tmp_int);
    }

    model::model(const Eigen::Vector3f& lower_bound, const Eigen::Vector3f& upper_bound, float grid_space, int num_threads, ros::NodeHandle nh)
    {
        nh_=nh;
        num_threads_ = num_threads;
        grid_spacing = grid_space;
        range_down = (lower_bound.array()/grid_spacing).floor().array();
        range_up = (upper_bound.array()/grid_spacing).ceil().array();
        num_bins = ((range_up-range_down).array()).cast <int> ();
        range_down *= grid_spacing;
        range_up *= grid_spacing;
        for (int i =0;i<3;i++) num_bins[i]++;
        // ROS_INFO_STREAM(num_bins);
        model_points.resize(num_bins.prod());
        // ROS_INFO_STREAM("bins "<<model_points.size());
        subscribeTopic();
    }

    int model::cart_to_model(Eigen::Vector3f pt) {
        // ROS_INFO_STREAM(pt);
        int z_bin = std::min(std::max(int(round((pt[2]-range_down[2])/grid_spacing)),0),num_bins[2]-1);
        int y_bin = std::min(std::max(int(round((pt[1]-range_down[1])/grid_spacing)),0),num_bins[1]-1);
        int x_bin = std::min(std::max(int(round((pt[0]-range_down[0])/grid_spacing)),0),num_bins[0]-1);
        // ROS_INFO_STREAM("pt "<<pt.transpose()<<", bins "<<x_bin<<","<<y_bin<<","<<z_bin<<":,"<<(pt[0]-range_down[0])/grid_spacing<<", "<<(pt[1]-range_down[1])/grid_spacing<<", "<<(pt[2]-range_down[2])/grid_spacing<<":,"<<num_bins.transpose());
        return z_bin*num_bins[1]*num_bins[0]+y_bin*num_bins[0]+x_bin;
    }

    void model::generate_thread(int start, int end, int thread_num) {
        // ROS_INFO_STREAM("thread "<<thread_num<<", "<<start<<", "<<end);
        for (int i=start;i<end;i++) {
            // ROS_INFO_STREAM(i);
            Eigen::VectorXf tmp_pts = avoid_pts_[i];
            bool success = true;
            // ROS_INFO_STREAM("i:"<<i<<"/"<<avoid_pts_.size()<<" ranges:"<<range_down.transpose()<<":"<<range_up.transpose()<<", tmp_pts:"<<tmp_pts.size());
            for (int k=0;k<3;k++) {
                if ((tmp_pts[k]>range_up[k])||(tmp_pts[k]<range_down[k])) {
                    success = false;
                    break;
                }
            }
            if (!success) continue;
            
            // ROS_INFO_STREAM("thread "<<thread_num<<", "<<tmp_pts.transpose());
            Eigen::Vector3f tmp_interval={tmp_pts[3], tmp_pts[4], tmp_pts[5]};
            // ROS_INFO_STREAM(tmp_pts.transpose());
            int idx = cart_to_model(tmp_pts.head(3));
            mtx.lock();
            // if ((model_points[idx].position[0]>-5)&&(model_points[idx].position[0]<5)) {
            //     if ((tmp_pts[0]!=model_points[idx].position[0])||(tmp_pts[1]!=model_points[idx].position[1])||(tmp_pts[2]!=model_points[idx].position[2])) {
            //         ROS_INFO_STREAM("error "<<tmp_pts.head(3).transpose()<<", "<<model_points[idx].position.transpose());
            //         int tmp = cart_to_model(mode(num_bins[1]*num_bins[0])l_points[idx].position);
            //     }
            // }
            // int z_bin = floor(idx/(num_bins[1]*num_bins[0]));
            // int y_bin = floor((idx-z_bin*(num_bins[1]*num_bins[0]))/num_bins[0]);
            // int x_bin = idx-z_bin*(num_bins[1]*num_bins[0])-y_bin*num_bins[0];
            // Eigen::Vector3f pos(x_bin,y_bin,z_bin);
            // pos = pos*grid_spacing+range_down;
            // ROS_INFO_STREAM("start pos:"<<tmp_pts.head(3).transpose()<<", desc pos:"<<pos.transpose());
            model_points[idx].position = tmp_pts.head(3);
            model_points[idx].avoidance_intervals_.push_back(tmp_interval);
            if ((tmp_pts[5]) && (tmp_pts[3]<model_points[idx].last_pass_time_)) {
                model_points[idx].last_pass_time_ = tmp_pts[3];
            }
            model_points[idx].allows_passage_ = model_points[idx].allows_passage_ and tmp_pts[5];

            mtx.unlock();
        }
    }
    void model::generate_model_cloud(std::vector<Eigen::VectorXf> avoid_pts) {

        pts_mtx.lock();
        avoid_pts_ = avoid_pts;
        ROS_INFO_STREAM("loop points "<<avoid_pts.size());
        pts_mtx.unlock();
        clear_model_cloud();
        if (!avoid_pts_.empty()) {
            generate_model_cloud();
        }
    }


    void model::clear_model_cloud(void) {
        std::lock_guard<std::mutex> lock(ext_mutex);
        for (int i:model_pt_idx) {
            model_points[i].avoidance_intervals_.clear();
        }

        model_pt_idx.clear();
        avoid_cart_pts.resize(3,0);
        joint_seq.clear();
    }

    void model::generate_model_cloud(void) {
        model_point tmp_point;
        Eigen::Vector3f tmp_interval;
        int idx = 0;
        int num_pts = static_cast<int>(avoid_pts_.size());
        // ROS_INFO_STREAM("loop points "<<num_pts);
        int pts_per_thread = ceil((float)num_pts/(float)num_threads_);
        threads.resize(num_threads_);
        for (int i = 0;i<num_threads_;i++) {
            // ROS_INFO_STREAM("starting thread "<<i);
            threads.at(i)=std::thread(&model::generate_thread,this,i*pts_per_thread,std::min((i+1)*pts_per_thread,num_pts),i);
        }
        for (int i=0;i<num_threads_;i++)
        {
            // ROS_INFO_STREAM("joining thread "<<i);
            if (threads.at(i).joinable())
            threads.at(i).join();
        }

        std::lock_guard<std::mutex> lock(ext_mutex);
        model_pt_idx.clear();
        avoid_cart_pts.resize(3,model_points.size());
        int j = 0;
        for (int i=0;i<model_points.size();i++) {
            if (!model_points[i].avoidance_intervals_.empty()) {
                model_points[i].merge_intervals();
                model_pt_idx.push_back(i);
                avoid_cart_pts.col(j) = model_points[i].position;
                j++;
            }
        }
        if (model_pt_idx.empty()) {
            avoid_cart_pts.resize(0,0);
        } else {
            avoid_cart_pts.conservativeResize(3,model_pt_idx.size());
        }
    }

    void model::subscribeTopic()
    {
        std::string human_model_topic;
        if (!nh_.getParam("human_model_topic",human_model_topic))
        {
            ROS_DEBUG("human_model_topic is not defined, using /human_model");
            human_model_topic="/human_model";
        }
        ROS_INFO("subscribe topic %s",human_model_topic.c_str());

        poses_sub=nh_.subscribe<std_msgs::Float32MultiArray>(human_model_topic,1,&model::humanCallback,this);

        std::string model_display_req_topic;
        if (!nh_.getParam("model_display_req_topic",model_display_req_topic))
        {
            ROS_DEBUG("human_model_topic is not defined, using /model_disp_req");
            model_display_req_topic="/model_disp_req";
        }
        ROS_INFO("subscribe topic %s",model_display_req_topic.c_str());

        disp_sub=nh_.subscribe<std_msgs::Float32>(model_display_req_topic,1,&model::displayRequestCallback,this);
        
        disp_all_sub=nh_.subscribe<std_msgs::Bool>("/model_disp_all_req",1,&model::displayAllRequestCallback,this);

        std::string model_display_pub_topic;
        if (!nh_.getParam("model_display_pub_topic",model_display_pub_topic))
        {
            ROS_DEBUG("human_marker_topic is not defined, using /human_markers");
            model_display_pub_topic="/human_markers";
        }
        disp_pub = nh_.advertise<visualization_msgs::Marker>(model_display_pub_topic,0,true);

        std::string human_seq_topic;
        if (!nh_.getParam("human_seq_topic",human_seq_topic))
        {
            ROS_DEBUG("human_seq_topic is not defined, using /human_model_seq");
            human_seq_topic="/human_model_seq";
        }
        sub_seq = nh_.subscribe<avoidance_intervals::joint_seq>(human_seq_topic,1,&model::subscribe_sequence,this);

    }

    void model::callbackThread(int start, int end, int h) {
        
        Eigen::VectorXf tmp_vec(h);
        for (int i=start;i<end;i++) {
            // ROS_INFO_STREAM("i:"<<i<<", start/end:"<<start<<"/"<<end);
            for (int j=0;j<h;j++) {
            tmp_vec[j]=msg_data[i*h+j];
            }
            // ROS_INFO_STREAM("tmp vec:"<<tmp_vec.transpose());
            avoid_pts_msg[i] = tmp_vec;
        }
    }

    void model::humanCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        ROS_INFO_STREAM("reading human----------------------------");

        //dim[0] is the number of quat elements per pose
        //dim[1] is the number of time steps predicted into the future
        //get the dimensions from the message
        // float dstride0 = msg->prediction.layout.dim[0].stride;
        // float dstride1 = msg->prediction.layout.dim[1].stride;
        avoid_pts_msg.clear();
        if (!msg->data.empty()) {
            int h = (int)msg->layout.dim[0].size;
            int w = (int)msg->layout.dim[1].size;
            //cast the msg data to a vector of floats
            std::lock_guard<std::mutex> lock(msg_mtx);
            msg_data = msg->data;
            avoid_pts_msg.resize(w);
            int num_threads_=30;
            int pts_per_thread = ceil((float)w/(float)num_threads_);
            std::cout<<"pts per thread:"<<pts_per_thread<<std::endl;
            std::vector<std::thread> threads;
            threads.resize(num_threads_);
            for (int i = 0;i<num_threads_;i++) {
                // ROS_INFO_STREAM("starting thread "<<i);
                threads.at(i)=std::thread(&model::callbackThread,this,i*pts_per_thread,std::min((i+1)*pts_per_thread,w),h);
            }
            for (int i=0;i<num_threads_;i++)
            {
                // ROS_INFO_STREAM("joining thread "<<i);
                if (threads.at(i).joinable())
                threads.at(i).join();
            }

            // for (int i=0;i<w;i++){
            //   for (int j=0;j<h;j++) [
            //     tmp_vec[j]=data[i*h+j];
            //   }
            //   avoid_pts[i]=tmp_vec;
            // }
        }
        generate_model_cloud(avoid_pts_msg);
        ROS_INFO_STREAM("updated the human model........................................"<<avoid_pts_msg.size());
    }

    void model::displayRequestCallback(const std_msgs::Float32::ConstPtr& msg) {
        // ROS_INFO_STREAM("display req callback----------------------------");
        float t = msg->data;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 2;
        p.y = 0;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 0;
        p.y = 1;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 0;
        p.y = 0;
        p.z = 0.2;
        marker.points.push_back(p);

        for(model_point mp:model_points) {
            for (Eigen::Vector3f a:mp.avoidance_intervals_){
                if ((a[0]<=t)&&(t<=a[1])) {
                    p.x = mp.position[0];
                    p.y = mp.position[1];
                    p.z = mp.position[2];
                    marker.points.push_back(p);
                    break;
                }
            }
        }
        if (marker.points.size()>0) {
            disp_pub.publish(marker);
            // ROS_INFO_STREAM("skeleton markers published");
        }
    }

    void model::displayAllRequestCallback(const std_msgs::Bool::ConstPtr& msg) {
        displayAllRequest();
    }
    void model::displayAllRequest(void) {
        // ROS_INFO_STREAM("display all req callback----------------------------");
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = grid_spacing*2;
        marker.scale.y = grid_spacing*2;
        marker.scale.z = grid_spacing*2;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 2;
        p.y = 0;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 0;
        p.y = 1;
        p.z = 0;
        marker.points.push_back(p);
        p.x = 0;
        p.y = 0;
        p.z = 0.2;
        marker.points.push_back(p);

        for(int i:model_pt_idx) {
            p.x = model_points[i].position[0];
            p.y = model_points[i].position[1];
            p.z = model_points[i].position[2];
            marker.points.push_back(p);
            for (int j=0;j<model_points[i].avoidance_intervals_.size();j++) {
                // ROS_INFO_STREAM("pt:"<<model_points[i].position.transpose()<<", avd:"<<model_points[i].avoidance_intervals_[j].transpose());
            }
        }
        // for(model_point pt:model_points) {
        //     if ((pt.position[0]>-5) && (pt.position[0]<5)) {
        //         p.x = pt.position[0];
        //         p.y = pt.position[1];
        //         p.z = pt.position[2];
        //         marker.points.push_back(p);
        //     }
        // }

        if (marker.points.size()>0) {
            disp_pub.publish(marker);
            // ROS_INFO_STREAM("skeleton markers published");
        }
    }

    void model::subscribe_sequence(const avoidance_intervals::joint_seq::ConstPtr& msg) {
        std::cout<<"receiving the sequence\n";
        joint_seq.clear();
        for (int i=0;i< msg->sequence.size();i++) {
            Eigen::MatrixXd joint_pos(3,msg->sequence[i].joint_pos.data.size());
            int c = 0;
            for (int j=0;j<msg->sequence[i].joint_pos.data.size();j++) {
                joint_pos.col(c)[j%3] = msg->sequence[i].joint_pos.data[j];
                if ((j>0)&&(j%3==0)) c++;
            }
            joint_seq.push_back(std::pair<float,Eigen::MatrixXd>(msg->sequence[i].time.data,joint_pos));
        }
    }

    void merge_intervals(std::vector<Eigen::Vector3f> intervals, std::vector<Eigen::Vector3f> &combined_intervals, float &last_pass_time ) {
        // ROS_INFO_STREAM("intervals size:"<<intervals.size());
        last_pass_time = std::numeric_limits<float>::infinity();
        if (intervals.empty()) return;
        std::sort(intervals.begin(),intervals.end(),sort_avoidance_intervals);
        float min_t = intervals[0][0];
        float max_t = intervals[0][1];
        bool is_last_pass = intervals[0][2];
        if (is_last_pass) last_pass_time = min_t;
        combined_intervals.clear();
        Eigen::Vector3f tmp_int;

        if (intervals.size()>1) {
            for (int i=0;i<int(intervals.size());i++) {
                // std::cout<<"min t"<<min_t<<", max t:"<<max_t<<"intervals:"<<intervals[i].transpose()<<std::endl;
                if (bool(intervals[i][2])) {
                    last_pass_time = min_t;
                    // ROS_INFO_STREAM("setting last pass time "<<last_pass_time);
                }
                if ((intervals[i][0]<min_t) && (intervals[i][0]<max_t)) {
                    min_t = intervals[i][0];
                    is_last_pass = is_last_pass && intervals[i][2];
                }
                if ((intervals[i][1]>max_t) && (intervals[i][0]<max_t)) {
                    max_t = intervals[i][1];
                }
                if (intervals[i][0]>max_t) {
                    tmp_int << min_t,max_t,is_last_pass;
                    // std::cout<<"tmp_int:"<<tmp_int.transpose()<<std::endl;
                    combined_intervals.push_back(tmp_int);
                    is_last_pass = false;
                    min_t = intervals[i][0];
                    max_t = intervals[i][1];
                    is_last_pass = intervals[i][2];
                }
            }
        }

        tmp_int << min_t,max_t,is_last_pass;
        // std::cout<<"tmp_int:"<<tmp_int.transpose()<<std::endl;
        combined_intervals.push_back(tmp_int);
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
        // std::cout<<"points\n";
        // for (int i=0;i<pts.cols();i++) {
        //     for (int j = 0;j<pts.rows();j++) {
        //         std::cout<<pts(j,i)<<",";
        //     }
        //     std::cout<<std::endl;
        // }
        pts = pts.array().round().matrix();
        pts=pts*grid_size;
        // std::cout<<"points\n";
        // for (int i=0;i<pts.cols();i++) {
        //     for (int j = 0;j<pts.rows();j++) {
        //         std::cout<<pts(j,i)<<",";
        //     }
        //     std::cout<<std::endl;
        // }
        // std::cin.ignore();
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
                // std::cout<<t_pt.transpose()<<std::endl;
                pts_t_vec.push_back(t_pt);
            }
        }
        return pts_t_vec;
    }

    skeleton::skeleton(ros::NodeHandle nh, const std::string topic_name,double grid_size, double t_step) {
        num_threads_ = 20;
        sub_skeletons = nh.subscribe(topic_name,1,&skeleton::callback, this);
        grid_size_ = grid_size;
        t_step_ = t_step;
        nh_=nh;
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "/human_markers2", 0 );    
        std::string human_model_topic;
        if (!nh_.getParam("human_model_topic",human_model_topic))
        {
            ROS_DEBUG("human_model_topic is not defined, using /human_model");
            human_model_topic="/human_model";
        }   
        std::string human_seq_topic;
        if (!nh_.getParam("human_seq_topic",human_seq_topic))
        {
            ROS_DEBUG("human_seq_topic is not defined, using /human_model_seq");
            human_seq_topic="/human_model_seq";
        }
        pts_pub_ = nh_.advertise<std_msgs::Float32MultiArray>( human_model_topic, 0, true);
        seq_pub = nh_.advertise<avoidance_intervals::joint_seq>( human_seq_topic, 0, true);
    }

    void skeleton::forward_kinematics(std::vector<float> pose_elements,std::vector<double> t_steps, int idx) {
        // ROS_INFO_STREAM("forward kinematics "<<pose_elements.size());
        // if (prediction.data[31]>0) {
        //     ROS_INFO_STREAM("error");
        //     ROS_INFO_STREAM(prediction);
        // }
        Eigen::Vector3f pelvis_loc = {pose_elements[1],pose_elements[2],pose_elements[3]};
        pelvis_loc = transform_to_world*pelvis_loc;

        Eigen::Quaternionf z_axis_quat(0,0,0,1);
        std::vector<Eigen::Quaternionf> quats;
        Eigen::Quaternionf q;

        Eigen::Quaternionf quat_to_world(transform_to_world.rotation());
        for (int i=0;i<7;i++){
            q = quat_to_world*Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]);
            quats.push_back(q);
            // ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
        }
        std::vector<Eigen::Vector3f> joint_locations;
        joint_locations.push_back(pelvis_loc);
        Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
        Eigen::Vector3f spine_top = pelvis_loc+link_lengths_[0]*z_spine.vec();
        joint_locations.push_back(spine_top);
        // ROS_INFO_STREAM("spine top "<<spine_top.transpose());
        Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
        Eigen::Vector3f head = spine_top+link_lengths_[1]*z_neck.vec();
        joint_locations.push_back(head);
        // ROS_INFO_STREAM("head top "<<head.transpose());
        Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
        Eigen::Vector3f l_shoulder = spine_top-0.5*link_lengths_[2]*z_shoulders.vec();
        joint_locations.push_back(l_shoulder);
        // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
        Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
        Eigen::Vector3f e1 = l_shoulder+link_lengths_[3]*z_e1.vec();
        joint_locations.push_back(e1 );
        Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
        Eigen::Vector3f w1 = e1+(link_lengths_[4]+0.1)*z_w1.vec();
        joint_locations.push_back(w1);
        Eigen::Vector3f r_shoulder = spine_top+0.5*link_lengths_[2]*z_shoulders.vec();
        joint_locations.push_back(r_shoulder);
        // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
        Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
        Eigen::Vector3f e2 = r_shoulder+link_lengths_[5]*z_e2.vec();
        joint_locations.push_back(e2);
        Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
        Eigen::Vector3f w2 = e2+(link_lengths_[6]+0.1)*z_w2.vec();
        joint_locations.push_back(w2);

        joint_seq[idx] = std::pair<float,std::vector<Eigen::Vector3f>>(pose_elements[0],joint_locations);
        quat_seq[idx] = std::pair<float,std::vector<Eigen::Quaternionf>>(pose_elements[0],quats);

        std::vector<Eigen::Matrix3f> rotations;
        std::vector<Eigen::MatrixXf> all_pts;
        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f r_y=Eigen::Matrix3f::Identity();
        r_y= Eigen::Quaternionf(0,0,1,0);//*Eigen::Quaternionf(0.707,0,0,0.707);
        
        for (int i=0;i<7;i++){
            r = Eigen::Quaternionf(quats[i]);
            rotations.push_back(r);
            // ROS_INFO_STREAM("rotation \n"<<r);
        }

        Eigen::MatrixXf oversampled_pts = (rotations[0]*raw_limb_points[0]).colwise() + pelvis_loc;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        // ROS_INFO_STREAM("pose torso \n"<<rotations[0]);
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);

        // Eigen::Vector3f spine_top = pelvis_loc+rotations[0]*Eigen::Vector3f(0,0,link_lengths_[0]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[1]*raw_limb_points[1]).colwise() + spine_top;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        // ROS_INFO_STREAM("pose neck \n"<<rotations[1]);

        // Eigen::Vector3f shoulder_vec=rotations[2]*Eigen::Vector3f(0,0,0.5*link_lengths_[2]);
        
        // Eigen::Vector3f origin = spine_top-shoulder_vec;
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[3]*raw_limb_points[2]).colwise() + l_shoulder;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        // ROS_INFO_STREAM("pose l up \n"<<rotations[3]);

        // origin = origin+rotations[3]*Eigen::Vector3f(0,0,link_lengths_[3]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[4]*raw_limb_points[3]).colwise()  + e1;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        // ROS_INFO_STREAM("pose l fore \n"<<rotations[4]);
                
        // origin = spine_top+shoulder_vec;
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[5]*raw_limb_points[4]).colwise() + r_shoulder;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        // ROS_INFO_STREAM("pose r up \n"<<rotations[5]);

        // origin = origin+rotations[5]*Eigen::Vector3f(0,0,link_lengths_[5]);
        oversampled_pts = Eigen::MatrixXf();
        oversampled_pts = (rotations[6]*raw_limb_points[5]).colwise() + e2;
        // oversampled_pts=transform_to_world.rotation()*oversampled_pts+transform_to_world.translation();
        downsample_points(oversampled_pts,grid_size_);
        all_pts.push_back(oversampled_pts);
        // ROS_INFO_STREAM("pose r fore \n"<<rotations[6]);
        // ROS_INFO_STREAM("end");
        // visualization_msgs::Marker marker;
        // // display_markers(all_pts,marker);
        // if (marker.points.size()>0) vis_pub_.publish(marker);
        // ROS_INFO_STREAM("waiting for enter");
        // std::cin.ignore();
        std::vector<Eigen::Vector4f> reduced_pts = remove_duplicates_add_time(all_pts,pose_elements[0],t_steps,t_step_);
        std::lock_guard<std::mutex> lock(_mtx_read);
        transformed_points.insert(transformed_points.end(),reduced_pts.begin(),reduced_pts.end());
        
    }

    void skeleton::callback(const human_motion_prediction::human_pose::ConstPtr& msg) {
        link_lengths_ = std::vector<float>(msg->link_lengths.data);
        link_radii_ = std::vector<float>(msg->link_radii.data);

        if (raw_limb_points.empty()){
            for (int i=0;i<link_lengths_.size();i++) {
                // std::cout<<link_lengths_[i]<<", "<<link_radii_[i]<<", ";
                if (i==2) continue;
                Eigen::MatrixXf tmp_pts = generate_cylinder_pts(link_radii_[i],link_lengths_[i],grid_size_);
                raw_limb_points.push_back(tmp_pts);
            }
            // std::cout<<std::endl;
            // visualization_msgs::Marker marker;
            // display_markers(raw_limb_points,marker);
            // if (marker.points.size()>0) vis_pub_.publish(marker);
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
        float tmp_time = 0;
        for (int i=0;i<w;i++){
            std::vector<double> t_steps;
            tmp_time = std::max(tmp_time,data[i*h]);
            //compute the human points via forward kinematics
            if (w>0) {
                if (i>0) {
                    t_steps.push_back(data[i*h]);
                }
                if (i<w-1) {
                    t_steps.push_back(data[(i+1)*h]);
                }
                forward_kinematics(std::vector<float>(data.begin()+i*h,data.begin()+(i+1)*h-1),t_steps,i);
            }
        }
        // ROS_INFO_STREAM("t2");
        std::lock_guard<std::mutex> l(_mtx);
        ready_points = transformed_points;
        end_time_ = tmp_time;
        std::vector<Eigen::VectorXf> avoid_pts = determine_intervals(transformed_points,end_time_,1);
        ready = true;
        
    }

    void skeleton::publish_pts(std::vector<Eigen::VectorXf> pts) {
        std_msgs::Float32MultiArray pts_msg;
        if (!pts.empty()) {
            pts_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            pts_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            pts_msg.layout.dim[0].size = pts[0].size();
            pts_msg.layout.dim[1].size = pts.size();
            int dstride1 = (int)pts_msg.layout.dim[1].size;
            pts_msg.layout.dim[0].stride = pts_msg.layout.dim[0].size*pts_msg.layout.dim[1].size;
            pts_msg.layout.dim[1].stride = dstride1;
            pts_msg.layout.data_offset = 0;
            pts_msg.data.resize(pts_msg.layout.dim[0].stride);
            for (int i = 0; i < pts_msg.layout.dim[1].size;i++) {
                for (int j = 0;j<pts_msg.layout.dim[0].size;j++) {
                    pts_msg.data[pts_msg.layout.dim[0].size*i+j] = pts[i][j];
                }
            }
        }
        pts_pub_.publish(pts_msg);
        // ROS_INFO_STREAM("published pts on "<<pts_pub_.getTopic());
    }

    void display_markers(std::vector<Eigen::Vector4f> pts,int t,double t_step, visualization_msgs::Marker &marker) {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
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
            if (pt[3]==t*t_step) {
                p.x = pt[0];
                p.y = pt[1];
                p.z = pt[2];
                marker.points.push_back(p);
            }
        }
        
    }
    
    void display_markers(std::vector<Eigen::Vector4f> pts,visualization_msgs::Marker &marker) {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
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
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            marker.points.push_back(p);
        }
        
    }

    void display_markers(std::vector<Eigen::VectorXf> pts,visualization_msgs::Marker &marker) {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.points.clear();
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
        for (Eigen::VectorXf pt:pts) {
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            marker.points.push_back(p);
        }
        
    }

    void display_markers(std::vector<Eigen::MatrixXf> pts,visualization_msgs::Marker &marker) {
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();//ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        // marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
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
        for (Eigen::MatrixXf pt_mat:pts) {
            for (int i = 0;i<pt_mat.cols();i++) {
                Eigen::Vector3f pt = pt_mat.col(i);
                p.x = pt[0];
                p.y = pt[1];
                p.z = pt[2];
                marker.points.push_back(p);
            }
        }
        
    }

    void skeleton::read_thread(std::vector<std::string> in_lines, std::string prev_line, std::string next_line,int thread_num, int start_id) {
        bool first_line = true;
        bool last_line = false;
        double last_time = 0.0;
        std::vector<float> data;
        std::stringstream ss=std::stringstream(in_lines[0]); 
        std::string substr;
        std::getline(ss,substr,',');     
        double t0 = std::stof(substr.c_str());
        double time_diff = 0.0;
        if (int(in_lines.size())>1) {
            ss=std::stringstream(in_lines[1]); 
            double t1 = std::stof(substr.c_str());
            time_diff = t1-t0;
        }
        std::vector<double> t_steps; 
        if (!prev_line.empty()) {
            first_line = false;
            ss=std::stringstream(prev_line);
            std::getline(ss,substr,',');
            last_time = std::stof(substr.c_str());
        }
        if (!next_line.empty()) {
            last_line=true;
        }
        for (int i= 0;i<in_lines.size();i++) {
            data.clear();
            ss=std::stringstream(in_lines[i]);
            std::string substr;
            while (std::getline(ss,substr,',')) {
                data.push_back(std::stof(substr.c_str()));
            }   
            t_steps.clear();   
            // std::cout<<"thread "<<thread_num<<":";
            if (!first_line) {
                t_steps.push_back(last_time);
                // std::cout<<"last time:"<<last_time<<", ";
            }
            // std::cout<<"this time:"<<data[0]<<", ";  
            if ((!last_line)||(i<in_lines.size()-1)) {
                t_steps.push_back(data[0]+time_diff);
                // std::cout<<"next time:"<<data[0]+time_diff<<", ";
            }
            // std::cout<<std::endl;
            forward_kinematics(std::vector<float>(data.begin(),data.end()),t_steps,start_id+i);
            time_diff = data[0]-last_time;
            last_time=data[0];
        }
        if (!last_line) end_time_ = data[0];
    }

    std::vector<Eigen::VectorXf> skeleton::read_human_task(int task_num, Eigen::Isometry3f transform) {
        transform_to_world = transform;
        if (raw_limb_points.empty()){
            for (int i=0;i<link_lengths_.size();i++) {
                if (i==2) continue;
                Eigen::MatrixXf tmp_pts = generate_cylinder_pts(link_radii_[i],link_lengths_[i],grid_size_);
                raw_limb_points.push_back(tmp_pts);
            }
        }
        transformed_points.clear();
        std::string file_name = ros::package::getPath("human_motion_prediction")+"/task_"+std::to_string(task_num) + "_t_means.csv";
        std::ifstream myfile (file_name); 
        std::string line;
        std::string prev_line;
        std::vector<std::string> all_lines;
        std::vector<std::thread> threads(num_threads_);

        if (myfile.is_open()) { 
            // ROS_INFO_STREAM("reading file ");
            while (std::getline(myfile,line)) {
                all_lines.push_back(line);
            }

            ROS_INFO_STREAM("all lines "<<all_lines.size());
            myfile.close();
            int num_lines = all_lines.size();
            joint_seq.resize(num_lines);
            quat_seq.resize(num_lines);
            int num_lines_per_thread = ceil((float)num_lines/(float)num_threads_);
            for (int i = 0;i<num_threads_;i++) {
                std::string prev_line;
                std::string next_line;
                bool last_line = ((i+1)*num_lines_per_thread>=num_lines) || (i==num_threads_-1);
                if (i>0) prev_line = all_lines[i*num_lines_per_thread-1];
                // if (i<num_threads_-1) next_line = all_lines[(i+1)*num_lines_per_thread];
                if (!last_line) next_line = all_lines[(i+1)*num_lines_per_thread];
                ROS_INFO_STREAM("starting thread "<<i<<", start line "<<i*num_lines_per_thread<<", last line "<<std::min((i+1)*num_lines_per_thread,num_lines));
                std::cout<<std::vector<std::string>(all_lines.begin(),all_lines.begin()+std::min((i+1)*num_lines_per_thread,num_lines)).size()<<std::endl;
                threads.at(i)=std::thread(&skeleton::read_thread,this,std::vector<std::string>(all_lines.begin()+i*num_lines_per_thread,all_lines.begin()+std::min((i+1)*num_lines_per_thread,num_lines)),prev_line,next_line,i,i*num_lines_per_thread);
                if (last_line) break;
            }
            for (int i=0;i<num_threads_;i++)
            {
                // ROS_INFO_STREAM("joining thread "<<i);
                if (threads.at(i).joinable())
                threads.at(i).join();
            }
            
            // ROS_INFO_STREAM("t2");
            std::lock_guard<std::mutex> l(_mtx);
            ready_points = transformed_points;
            visualization_msgs::Marker marker;
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>("/human_markers", 0, true);
            // ROS_INFO_STREAM("end time:"<<end_time_);
            std::vector<Eigen::VectorXf> avoid_pts = determine_intervals(transformed_points,end_time_,1,marker);
            pub.publish(marker);
            // ROS_INFO_STREAM("published markers");
            // std::cin.ignore();
            // marker.points.clear();

            // pub.publish(marker);
            // ROS_INFO_STREAM("press enter");
            // std::cin.ignore();
            // display_markers(avoid_pts,marker);
            // pub.publish(marker);
            // ROS_INFO_STREAM("published markers");
            // std::cin.ignore();

            ready = true;
            return avoid_pts;
        }

    }

    void skeleton::publish_sequence(double start_time) {
        avoidance_intervals::joint_seq joint_sequence_msg;
        avoidance_intervals::joint_seq_elem joint_seq_elem_msg;
        int start_i = std::min(int(floor(start_time/t_step_)),int(joint_seq.size())-1);
        std::cout<<"publishing the sequence of length:"<<int(joint_seq.size())-start_i<<" starting from step "<<start_i<<std::endl;
        for (int i=start_i;i<joint_seq.size();i++) {
            joint_seq_elem_msg.time.data = joint_seq[i].first;
            joint_seq_elem_msg.joint_pos.data.clear();
            for (int j=0;j<joint_seq[i].second.size();j++) {
                for (int k=0;k<3;k++) {
                    joint_seq_elem_msg.joint_pos.data.push_back(joint_seq[i].second[j][k]);
                }
            }
            joint_sequence_msg.sequence.push_back(joint_seq_elem_msg);
        }
        seq_pub.publish(joint_sequence_msg);
    }

    geometry_msgs::PoseArray skeleton::get_pose_at_time(double t) {
        geometry_msgs::PoseArray poses;
        geometry_msgs::Pose p;
        int idx = round(t/t_step_);
        if (idx>int(joint_seq.size())-1) return poses;
        for (int j=0;j<joint_seq[idx].second.size();j++) {
            p.position.x = joint_seq[idx].second[j][0];
            p.position.y = joint_seq[idx].second[j][1];
            p.position.z = joint_seq[idx].second[j][2];
            poses.poses.push_back(p);
        }
        return poses;
    }
    std::vector<Eigen::Quaternionf> skeleton::get_quats_at_time(double t) {
        int idx = round(t/t_step_);
        if (idx>int(joint_seq.size())-1) return std::vector<Eigen::Quaternionf>();
        return quat_seq[idx].second;
    }

}