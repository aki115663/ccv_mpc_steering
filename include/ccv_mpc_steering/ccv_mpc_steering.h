
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
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Pose2D.h>

//ipopt
#include <Eigen/Core>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>

class CcvMpcSteering
{
public:
    CcvMpcSteering();
    ~CcvMpcSteering();
    void process();

private:

    int hz_;
    double PITCH_OFFSET_;
    // int NX = 4 // x = x, y, v, yaw
    // int NU = 2  // a = [accel, steer]
    // int T = 5  // horizon length
    //
    // // mpc parameters
    // R = np.diag([0.01, 0.01])  // input cost matrix
    // Rd = np.diag([0.01, 1.0])  // input difference cost matrix
    // Q = np.diag([1.0, 1.0, 0.5, 0.5])  // state cost matrix
    // Qf = Q  // state final matrix
    // GOAL_DIS = 1.5  // goal distance
    // STOP_SPEED = 0.5 / 3.6  // stop speed
    // MAX_TIME = 500.0  // max simulation time
    //
    // // iterative paramter
    // MAX_ITER = 3  // Max iteration
    // DU_TH = 0.1  // iteration finish param
    //
    // TARGET_SPEED = 10.0 / 3.6  // [m/s] target speed
    // N_IND_SEARCH = 10  // Search index number
    //
    // DT = 0.2  // [s] time tick
    //
    // // Vehicle parameters
    // LENGTH = 4.5  // [m]
    // WIDTH = 2.0  // [m]
    // BACKTOWHEEL = 1.0  // [m]
    // WHEEL_LEN = 0.3  // [m]
    // WHEEL_WIDTH = 0.2  // [m]
    // TREAD = 0.7  // [m]
    // WB = 2.5  // [m]
    //
    // MAX_STEER = np.deg2rad(45.0)  // maximum steering angle [rad]
    // MAX_DSTEER = np.deg2rad(30.0)  // maximum steering speed [rad/s]
    // MAX_SPEED = 55.0 / 3.6  // maximum speed [m/s]
    // MIN_SPEED = -20.0 / 3.6  // minimum speed [m/s]
    // MAX_ACCEL = 1.0  // maximum accel [m/ss]

    void predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg);
    void trajectory_marker_callback(const visualization_msgs::Marker::ConstPtr &msg);

    //member
    nav_msgs::Path predicted_path_;
    double target_velocity_;
    bool have_received_path_=false;
    bool have_reached_goal_=false;
    bool read_marker_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_local_goal_;
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_cmd_pos_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::Pose2D current_pose_;
    std::string world_frame_id_;
    std::string robot_frame_id_;


//#############################################################################
//#############################################################################
    // int hz_;
    // double MAX_TARGET_VELOCITY_;
    // double MAX_STEERING_ANGLE_;
    // double TREAD_;
    // double PITCH_OFFSET_;
    // double GOAL_BORDER_;
    //
    // void predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg);
    // void update_carrots();
    // void update_motion();
    // void update_target_velocity();
    // double calc_distance(const double &x1, const double &y1, const double &x2, const double &y2);
    // double calc_steering_angle(double r, double delta, double a, char side);
    // double calc_turn_radius(double r, double delta, double a, char side);
    // void flip_velocity(double &delta, double &velocity);
    //
    // //member
    // nav_msgs::Path predicted_path_;
    // double target_velocity_;
    // bool have_received_path_=false;
    // bool have_reached_goal_=false;
    // bool read_marker_;
    // ros::NodeHandle nh_;
    // ros::NodeHandle private_nh_;
    // ros::Subscriber sub_path_;
    // ros::Subscriber sub_local_goal_;
    // ros::Subscriber sub_marker_;
    // ros::Publisher pub_cmd_vel_;
    // ros::Publisher pub_cmd_pos_;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
    // geometry_msgs::Pose2D current_pose_;
    // std::string world_frame_id_;
    // std::string robot_frame_id_;
    //
    // //debug
    // ros::Publisher pub_carrot1_;
    // ros::Publisher pub_carrot2_;
    // ros::Publisher pub_cx_line_;
    // nav_msgs::Path cx_line_;
};
#endif //CCV_PURE_PURSUIT_STEERING_H
