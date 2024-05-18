#include <ccv_mpc_steering/ccv_mpc_steering.h>

using CppAD::AD;


MpcPathTracker::MpcPathTracker():private_nh_("~"), tf_listener_(tf_buffer_)
{
    // param
    private_nh_.param("hz", HZ_, {10});
    private_nh_.param("horizon_t", HORIZON_T_, {15});
    private_nh_.param("world_frame_id", world_frame_id_, {"odom"});
    private_nh_.param("robot_frame_id", robot_frame_id_, {"base_link"});
    private_nh_.param("tread", TREAD_, {0.5});
    private_nh_.param("wheel_radius", WHEEL_RADIUS_, {0.15});
    private_nh_.param("goal_border", GOAL_BORDER_, {0.3});
    private_nh_.param("pitch_offset", PITCH_OFFSET_, {3.0 * M_PI / 180.0});
    private_nh_.param("vref", VREF_, {1.2});
    private_nh_.param("resolution", RESOLUTION_, {0.1});
    private_nh_.param("max_velocity", MAX_VELOCITY_, {1.5});
    private_nh_.param("max_angular_velocity", MAX_ANGULAR_VELOCITY_, {1.0});
    private_nh_.param("max_wheel_angular_acceleration", WHEEL_ANGULAR_ACCELERATION_LIMIT_, {3.14});
    private_nh_.param("max_wheel_angular_velocity", WHEEL_ANGULAR_VELOCITY_LIMIT_, {1.57}); //要確認
    private_nh_.param("max_steering_angle", STEERING_ANGLE_LIMIT_, {20*M_PI/180});

    vars_bounds={MAX_VELOCITY_, MAX_ANGULAR_VELOCITY_, WHEEL_ANGULAR_ACCELERATION_LIMIT_, WHEEL_ANGULAR_VELOCITY_LIMIT_, STEERING_ANGLE_LIMIT_};


    // subscriber
    sub_path_ = nh_.subscribe("/local_path", 10, &MpcPathTracker::path_callback, this, ros::TransportHints().tcpNoDelay());
    sub_joint_states_ = nh_.subscribe("/sq2_ccv/joint_states", 10, &MpcPathTracker::joint_states_callback, this);
    // publisher
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

    path_x = Eigen::VectorXd::Zero(HORIZON_T_);
    path_y = Eigen::VectorXd::Zero(HORIZON_T_);
    path_yaw = Eigen::VectorXd::Zero(HORIZON_T_);
    
}


FG_eval::FG_eval(Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
    this->ref_x = ref_x;
    this->ref_y = ref_y;
    this->ref_yaw = ref_yaw;
}



void MpcPathTracker::path_callback(const nav_msgs::PathConstPtr &msg)
{
    std::cout << "path callback" << std::endl;
    have_received_path_ = true;
    path_ = *msg;
}

void MpcPathTracker::joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    //std::cout << "joint states callback" << std::endl;
    if (right_steering_joint_idx == std::numeric_limits<size_t>::max() || left_steering_joint_idx == std::numeric_limits<size_t>::max() ||
        right_wheel_joint_idx == std::numeric_limits<size_t>::max() || left_wheel_joint_idx == std::numeric_limits<size_t>::max())
    {
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "right_steering_joint") right_steering_joint_idx = i;
            else if (msg->name[i] == "left_steering_joint") left_steering_joint_idx = i;
            else if(msg->name[i] == "right_wheel_joint") right_wheel_joint_idx = i;
            else if(msg->name[i] == "left_wheel_joint") left_wheel_joint_idx = i;
        }
    }
    else
    {
        steering_angle_r = msg->position[right_steering_joint_idx];
        steering_angle_l = msg->position[left_steering_joint_idx];
        omega_r = msg->velocity[right_wheel_joint_idx];
        omega_l = msg->velocity[left_wheel_joint_idx];
    }
}
bool MpcPathTracker::update_current_pose()
{
    bool transformed = false;
    geometry_msgs::PoseStamped pose;
    try
    {
        geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));
        current_pose.header = tf.header;
        current_pose.pose.position.x = tf.transform.translation.x;
        current_pose.pose.position.y = tf.transform.translation.y;
        current_pose.pose.orientation = tf.transform.rotation;
        pose.header = current_pose.header;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.orientation = tf.transform.rotation;
        transformed = true;
    }
    catch (tf2::TransformException &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return transformed;
}

void MpcPathTracker::update_current_state(Eigen::VectorXd& state){
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time;
    last_time = current_time;
    double dx = current_pose.pose.position.x - previous_pose.pose.position.x;
    double dy = current_pose.pose.position.y - previous_pose.pose.position.y;
    double dyaw = tf2::getYaw(current_pose.pose.orientation) - tf2::getYaw(previous_pose.pose.orientation);
    double v = sqrt(dx * dx + dy * dy) / dt;
    double omega = dyaw / dt;
    //x, y, yaw, v, omega, omega_l, omega_r, steer_l, steer_r
    state << 0, 0, tf2::getYaw(current_pose.pose.orientation), v, omega, omega_l, omega_r, steering_angle_l, steering_angle_r;
}

void MpcPathTracker::path_to_vector(void){
    int m = VREF_ * DT_ / RESOLUTION_ + 1;// TODO:delete 1
    int index = 0;
    for(int i=0;i<HORIZON_T_;i++){
        if(i*m<path_.poses.size()){
            index = i*m;
            path_x[i] = path_.poses[index].pose.position.x;
            path_y[i] = path_.poses[index].pose.position.y;
            path_yaw[i] = tf2::getYaw(path_.poses[index].pose.orientation);
        }else{
            path_x[i] = path_.poses[path_.poses.size() - 1].pose.position.x;
            path_y[i] = path_.poses[path_.poses.size() - 1].pose.position.y;
            path_yaw[i] = tf2::getYaw(path_.poses[path_.poses.size() - 1].pose.orientation);
        }
    }
}



int MPC::solve(Eigen::VectorXd state, Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
// std::vector<double> MPC::solve(int HORIZON_T_, Eigen::VectorXd state, Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
    //state = {x, y, yaw, v, omega, omega_l, omega_r, steer_l, steer_r}
    bool ok = true;
    size_t i;

    double x = state[0];
    double y = state[1];
    double yaw = state[2];
    double v = state[3];
    double omega = state[4];
    double omega_l = state[5];
    double omega_r = state[6];
    double steer_l = state[7];
    double steer_r = state[8];

    // 7(x, y, yaw, v, omega, omega_l, omega_r, steer_l, steer_r), 4(domega_l, domega_r, dsteer_l, dsteer_r)
    size_t n_variables = 9 * T + 4 * (T - 1);
    size_t n_constraints = 9 * T;

    Dvector vars(n_variables);
    for(int i=0;i<n_variables;i++){
        vars[i] = 0.0;
    }
    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;
    vars[omega_start] = omega;
    vars[omega_l_start] = omega_l;
    vars[omega_r_start] = omega_r;
    vars[steer_l_start] = steer_l;
    vars[steer_r_start] = steer_r;

    Dvector vars_lower_bound(n_variables);
    Dvector vars_upper_bound(n_variables);

    for(int i=0;i<v_start;i++){
        // x, y, yaw
        vars_lower_bound[i] = -1.0e19;
        vars_upper_bound[i] = 1.0e19;
    }
    for(int i=v_start;i<omega_start;i++){
        // v
        vars_lower_bound[i] = 0;
        vars_upper_bound[i] = vars_bounds[0]; //MAX_VELOCIY_
    }
    for(int i=omega_start;i<omega_r_start;i++){
        // omega
        vars_lower_bound[i] = -vars_bounds[1]; //MAX_ANGULAR_VELOCITY
        vars_upper_bound[i] = vars_bounds[1];
    }
    for(int i=omega_r_start;i<domega_r_start;i++){
        // omega_r, omega_l
        vars_lower_bound[i] = -vars_bounds[3]; //WHEEL_ANGURAL_VELOCITY_LIMIT
        vars_upper_bound[i] = vars_bounds[3];
    }
    for(int i=omega_start;i<;i++){
        // domega_r, domega_l
        vars_lower_bound[i] = -vars_bounds[2]; //WHEEL_ANGURAL_ACCELERATION_LIMIT
        vars_upper_bound[i] = vars_bounds[2];
    }
    
    
    


    return 0;
}

    // double eps = 0.1/180*M_PI;
    // if(fabs(steering_angle_l)<eps && fabs(steering_angle_r)<eps){
    //     double omega_r = (v + omega * TREAD_ / 2.0) / WHEEL_RADIUS_;
    //     double omega_l = (v - omega * TREAD_ / 2.0) / WHEEL_RADIUS_;
    // }
    // else if(1)
    // {
    //     double turning_radius_l = sin(fabs(steering_angle_r))*TREAD_/sin(fabs(steering_angle_l - steering_angle_r));
    //     double turning_radius_r = sin(fabs(steering_angle_l))*TREAD_/sin(fabs(steering_angle_l - steering_angle_r));
    //     double omega_r = (v + omega * fabs(turning_radius_l - turning_radius_r) / 2.0) / WHEEL_RADIUS_;
    //     double omega_l = (v - omega * fabs(turning_radius_l - turning_radius_r) / 2.0) / WHEEL_RADIUS_;
    // }
    



    




void MpcPathTracker::process()
{
    ros::Rate loop_rate(HZ_);

    while (ros::ok())
    {
        if (have_received_path_)
        {
            bool transformed = update_current_pose();

            if (transformed && !path_.poses.empty())
            {
                std::cout << "=== ccv mpc steering ===" << std::endl;
                if (first_transform)
                {
                    last_time = ros::Time::now().toSec();
                    first_transform = false;
                }
                else
                {
                    MPC mpc(HORIZON_T_);
                    double current_time = ros::Time::now().toSec();
                    double dt = current_time - last_time;
                    last_time = current_time;

                    Eigen::VectorXd current_state(9);
                    update_current_state(current_state);
                    path_to_vector();
                    // std::cout<<"path_x :\n"<<path_x<<std::endl;
                    // std::cout<<"path_y :\n"<<path_y<<std::endl;
                    // std::cout<<"path_yaw:\n"<<path_yaw<<std::endl;
                    auto result=mpc.solve(current_state, path_x, path_y, path_yaw);
                }
                previous_pose = current_pose;
            }
        }
        else
            ROS_WARN_STREAM("cannot receive path");
        ros::spinOnce();
        std::cout << "==============================" << std::endl;
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_path_tracker");
    MpcPathTracker mpc_path_tracker;
    mpc_path_tracker.process();
    return 0;
}
