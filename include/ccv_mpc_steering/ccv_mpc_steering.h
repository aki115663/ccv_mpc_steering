#ifndef CCV_PURE_PURSUIT_STEERING_H
#define CCV_PURE_PURSUIT_STEERING_H

//ros
#include<ros/ros.h>
#include<ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include<nav_msgs/Path.h>  // Express velocity in terms of linear and angular components , Vector3
#include<geometry_msgs/Twist.h>
#include<dynamixel_workbench_msgs/DynamixelStateList.h>  // "dynamixel_state" is list type
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

//ipopt
#include <Eigen/Core>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>

class MPC
{
    public:
    MPC(int HORIZON_T_, std::vector<std::string> state, std::vector<std::string> input, std::vector<double> bound)
    {
        T=HORIZON_T_;
        //state
        // x_start = 0;
        // y_start = x_start + T;
        // yaw_start = y_start + T;
        // v_start = yaw_start + T;
        // omega_start = v_start + T;
        // omega_l_start = omega_start + T;
        // omega_r_start = omega_l_start + T;
        // steer_l_start = omega_r_start + T;
        // steer_r_start = steer_l_start + T;
        // //input
        // domega_l_start = steer_r_start + T;
        // domega_r_start = domega_l_start + T;
        // dsteer_l_start = domega_r_start + T;
        // dsteer_r_start = dsteer_l_start + T-1;
        // 7(x, y, yaw, v, omega, omega_l, omega_r, steer_l, steer_r), 4(domega_l, domega_r, dsteer_l, dsteer_r)

        this->state = state;
        this->input = input;
        this->bound = bound;
        
        
        
    };
    ~MPC(void){};

    // state, ref_x, ref_y, ref_yaw
    // std::vector<double> solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);
    int solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

    private:
    typedef CPPAD_TESTVECTOR(double) Dvector;
    int T;
    size_t x_start = 0;
    size_t y_start;
    size_t yaw_start;
    size_t v_start;
    size_t omega_start;
    size_t omega_l_start;
    size_t omega_r_start;
    size_t steer_l_start;
    size_t steer_r_start;
    size_t domega_l_start;
    size_t domega_r_start;
    size_t dsteer_l_start;
    size_t dsteer_r_start;
    std::vector<double> bound;
    std::vector<std::string> state;
    std::vector<std::string> input;

    int failure_count = 0;
    std::vector<double> result;

    void set_vars_bounds(Dvector);
};
class MpcPathTracker
{
public:
    MpcPathTracker(void);
    ~MpcPathTracker(void){};
    void process(void);

private:
    //周期
    double HZ_;
    double DT_=0.1;
    //ホライゾン長さ
    int HORIZON_T_;
    //目標速度
    double VREF_;
    //最大速度
    double MAX_VELOCITY_;
    //最大角速度
    double MAX_ANGULAR_VELOCITY_;
    //ホイール角加速度
    double WHEEL_ANGULAR_ACCELERATION_LIMIT_;
    //ホイール角速度
    double WHEEL_ANGULAR_VELOCITY_LIMIT_;
    //ステアリング角
    double STEERING_ANGLE_LIMIT_;
    //トレッド
    double TREAD_;
    //ホイール半径
    double WHEEL_RADIUS_;
    double PITCH_OFFSET_;
    double GOAL_BORDER_;
    //グリッドマップ分解能
    double RESOLUTION_;
    std::vector<double> vars_bound;

    void path_callback(const nav_msgs::Path::ConstPtr&);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr&);
    bool update_current_pose(void);
    void update_current_state(Eigen::VectorXd&);
    void path_to_vector(void);

    bool have_received_path_=false;
    bool first_transform=true;
    bool transformed = false;
    double last_time;
    double steering_angle_l;
    double steering_angle_r;
    double omega_l;
    double omega_r;
    Eigen::VectorXd path_x;
    Eigen::VectorXd path_y;
    Eigen::VectorXd path_yaw;
    size_t right_steering_joint_idx = std::numeric_limits<size_t>::max();
    size_t left_steering_joint_idx = std::numeric_limits<size_t>::max();
    size_t right_wheel_joint_idx = std::numeric_limits<size_t>::max();
    size_t left_wheel_joint_idx = std::numeric_limits<size_t>::max();
    ros::Time steer_sub_time;

    std::vector<std::string> mpc_states;
    std::vector<std::string> mpc_inputs;


    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_joint_states_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_cmd_pos_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::TransformStamped tf;
    nav_msgs::Path path_;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped previous_pose;
    geometry_msgs::Twist vel_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    std::string world_frame_id_;
    std::string robot_frame_id_;
};

class FG_eval{
public:
    FG_eval(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    void operator()(ADvector&, const ADvector&);

private:
    Eigen::VectorXd ref_x;
    Eigen::VectorXd ref_y;
    Eigen::VectorXd ref_yaw;

};
#endif //CCV_PURE_PURSUIT_STEERING_H



