#include<ccv_mpc_steering/ccv_mpc_steering.h>

CcvMpcSteering::CcvMpcSteering():private_nh_("~"), tf_listener_(tf_buffer_)
{
    //param
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("pitch_offset", PITCH_OFFSET_, {3.0*M_PI/180.0});
    private_nh_.param("")

    //subscriber
    sub_path_=nh_.subscribe("/predicted_path", 10, &CcvMpcSteering::predicted_trajectory_callback, this, ros::TransportHints().tcpNoDelay());
    //publisher
    pub_cmd_vel_=nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
    pub_cmd_pos_=nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

}


CcvMpcSteering::~CcvMpcSteering(){}


void CcvMpcSteering::predicted_trajectory_callback(const nav_msgs::Path::ConstPtr &msg)
{
    have_received_path_=true;
    predicted_path_=*msg;

    // geometry_msgs::TransformStamped tf;
    //
    // //Change the coordinate system of predicted_trajectory to odom
    // try
    // {
    //     tf = tf_buffer_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0));
    //     for(auto &pose : predicted_path_.poses)
    //     {
    //         pose.pose.position.x -= tf.transform.translation.x;
    //         pose.pose.position.y -= tf.transform.translation.y;
    //     }
    //     current_pose_.x = tf.transform.translation.x;
    //     current_pose_.y = tf.transform.translation.y;
    //     current_pose_.theta = std::asin(tf.transform.rotation.z)*2.0;
    // }
    // catch(tf2::TransformException ex)
    // {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    //     return;
    // }
}

void CcvMpcSteering::process()
{
    ros::Rate loop_rate(hz_);
    geometry_msgs::Twist vel;
    vel.linear.x=1.0;
    vel.angular.z=0.0;

    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
    cmd_pos.steer_r=0.0;
    cmd_pos.steer_l=0.0;
    cmd_pos.fore=PITCH_OFFSET_;
    cmd_pos.rear=PITCH_OFFSET_;
    cmd_pos.roll=0.0;

    while(ros::ok())
    {
        if(have_received_path_)
        {
            pub_cmd_vel_.publish(vel);
            pub_cmd_pos_.publish(cmd_pos);
        }
        else ROS_WARN_STREAM("cannot receive path");
        ros::spinOnce();
        std::cout<<"=============================="<<std::endl;
        loop_rate.sleep();
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ccv_mpc_steering");
    CcvMpcSteering ccv_mpc_steering;
    ccv_mpc_steering.process();
    return 0;
}
