/*
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class PathPublisher
{
public:
    PathPublisher();

private:
    void generatePath();
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
};

PathPublisher::PathPublisher()
{
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    generatePath();
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_pub_.publish(path_);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PathPublisher::generatePath()
{
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "odom";

    // Define waypoints
    for (int i = 0; i < 50; ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = i * 0.1;
        pose.pose.position.y = 2*sin(i * 0.1);
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1.0;

        path_.poses.push_back(pose);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    PathPublisher path_publisher;
    return 0;
}

*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class PathPublisher
{
public:
    PathPublisher();

private:
    void generatePath(int);
    void updateRobotPose();
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    tf::TransformListener listener_;
    geometry_msgs::PoseStamped current_pose_;
    std::string robot_frame_;
    std::string world_frame_;
};

PathPublisher::PathPublisher()
{
    nh_.param<std::string>("/dynamic_avoidance/ROBOT_FRAME", robot_frame_, "base_link");
    nh_.param<std::string>("/dynamic_avoidance/WORLD_FRAME", world_frame_, "odom");
    
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);

    ros::Rate loop_rate(10);
    int callback_loop=0;
    while (ros::ok())
    {   
        updateRobotPose();
        generatePath(callback_loop);
        path_pub_.publish(path_);
        ros::spinOnce();
        loop_rate.sleep();
        callback_loop++;
    }
}

void PathPublisher::updateRobotPose()
{
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), transform);
        current_pose_.header.stamp = transform.stamp_;
        current_pose_.header.frame_id = world_frame_;
        current_pose_.pose.position.x = transform.getOrigin().x();
        current_pose_.pose.position.y = transform.getOrigin().y();
        current_pose_.pose.position.z = transform.getOrigin().z();
        current_pose_.pose.orientation.x = transform.getRotation().x();
        current_pose_.pose.orientation.y = transform.getRotation().y();
        current_pose_.pose.orientation.z = transform.getRotation().z();
        current_pose_.pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

void PathPublisher::generatePath(int callback_loop)
{
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = world_frame_;

    double path_length = 5.0; // meters
    double path_resolution = 0.1; // meters
    int num_points = static_cast<int>(path_length / path_resolution);

    tf::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    path_.poses.clear();
    for (int i = 0; i < num_points; ++i)
    {
        double x = i * path_resolution;
        double y = 0.5 * sin(x); // Example path, sine wave

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = world_frame_;
        
        pose.pose.position.x = current_pose_.pose.position.x + x * cos(yaw+0.02*callback_loop) - y * sin(yaw);
        pose.pose.position.y = current_pose_.pose.position.y + x * sin(yaw+0.02*callback_loop) + y * cos(yaw);
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        path_.poses.push_back(pose);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    PathPublisher path_publisher;
    return 0;
}
