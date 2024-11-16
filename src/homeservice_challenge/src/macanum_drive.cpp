#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/QR>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

geometry_msgs::Twist twist_;

void twist_subscriber_callback(const geometry_msgs::Twist::ConstPtr& twist)
{
    twist_ = *twist;
    ROS_INFO_STREAM_THROTTLE(10, "cmd_vel changed. " << twist->linear.x << ", " << twist->angular.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mecanum_drive");
    ros::NodeHandle nh;

    ros::Publisher joint_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_cmd", 10);
    ros::Subscriber twist_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, twist_subscriber_callback);

    trajectory_msgs::JointTrajectory state;
    state.header.frame_id = "base";
    state.joint_names.push_back("wheel1");
    state.joint_names.push_back("wheel2");
    state.joint_names.push_back("wheel3");
    state.joint_names.push_back("wheel4");
    state.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    state.points.front().velocities.push_back(0.0);
    state.points.front().velocities.push_back(0.0);
    state.points.front().velocities.push_back(0.0);
    state.points.front().velocities.push_back(0.0);


    // mecanum wheel radius : 0.5 (subwheel was not considered, but it may ok)
    // l1 = 0.1125, l2 = 0.0925
    
    const double alpha = 0.1125 + 0.0925;
    Eigen::Matrix<double, 4, 3> macanum_kinematics_matrix;
    macanum_kinematics_matrix <<
        -1, -1, -alpha,
         1, -1, -alpha,
        -1, 1, -alpha,
         1, 1, -alpha;
    macanum_kinematics_matrix /= 0.05; // wheel radius

    while (ros::ok())
    {
        Eigen::Vector3d velocities(twist_.linear.x, twist_.linear.y, twist_.angular.z);
        Eigen::Vector4d angular_velocities =  macanum_kinematics_matrix * velocities;
        
        state.points.front().velocities[0] = angular_velocities(0);
        state.points.front().velocities[1] = angular_velocities(1);
        state.points.front().velocities[2] = angular_velocities(2);
        state.points.front().velocities[3] = angular_velocities(3);

        ROS_INFO_STREAM_THROTTLE(1,
            "Velocity published. " <<
            state.points.front().velocities[0] << ", " << state.points.front().velocities[1] << ", " <<
            state.points.front().velocities[2] << ", " << state.points.front().velocities[3]);        

        joint_cmd_pub_.publish(state);

        ros::spinOnce();
    }
}