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
        this->state = state;
        this->input = input;
        this->bound = bound;
        
    };
    ~MPC(void){};

                            // state,           ref_x,              ref_y,          ref_yaw
    std::vector<double> solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

    private:
    typedef CPPAD_TESTVECTOR(double) Dvector;
    int T;
    std::vector<double> bound;
    std::vector<std::string> state;
    std::vector<std::string> input;
    int failure_count = 0;
    std::vector<double> result;

    void set_vars_bounds(Dvector);
};

class FG_eval{
public:     //ref_x,              ref_y,          ref_yaw
    FG_eval(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    void operator()(ADvector&, const ADvector&);

private:
    int T=15;
    double DT=0.1;
    double VREF=1.2;
    double WHEEL_RADIUS=0.15;
    double TREAD=0.5;
    Eigen::VectorXd ref_x;
    Eigen::VectorXd ref_y;
    Eigen::VectorXd ref_yaw;

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
    //目標位置
    double X_BOUND_;
    double Y_BOUND_;
    double YAW_BOUND_;
    //最大速度
    double V_BOUND_;
    //最大角速度
    double OMEGA_BOUND_;
    //ホイール角速度
    double OMEGA_R_BOUND_;
    double OMEGA_L_BOUND_;
    //ステア角
    double STEER_R_BOUND_;
    double STEER_L_BOUND_;
    //ホイール角加速度
    double DOMEGA_R_BOUND_;
    double DOMEGA_L_BOUND_;
    //ステア角速度
    double DSTEER_R_BOUND_;
    double DSTEER_L_BOUND_;
    //トレッド
    double TREAD_;
    //ホイール半径
    double WHEEL_RADIUS_;
    double PITCH_OFFSET_;
    double GOAL_BORDER_;
    //グリッドマップ分解能
    double RESOLUTION_;

    std::vector<double> mpc_bound;
    std::vector<std::string> mpc_state;
    std::vector<std::string> mpc_input;
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

#endif //CCV_PURE_PURSUIT_STEERING_H



