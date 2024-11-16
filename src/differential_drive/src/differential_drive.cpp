#include <ros/ros.h>
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
    ros::init(argc, argv, "differential_controller");
    ros::NodeHandle nh;

    ros::Publisher joint_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_cmd", 10);
    ros::Subscriber twist_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, twist_subscriber_callback);

    trajectory_msgs::JointTrajectory state;
    state.header.frame_id = "base";
    state.joint_names.push_back("wheel1");
    state.joint_names.push_back("wheel2");
    state.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    state.points.front().positions.push_back(0.0);
    state.points.front().positions.push_back(0.0);
    state.points.front().velocities.push_back(0.0);
    state.points.front().velocities.push_back(0.0);

    // The wheel size was almost 66 mm when check using stl file.
    // r = 0.033 m
    //  left_wheel.origin = 0,  0.08, 0.023
    // right_wheel.origin = 0, -0.08, 0.023
    // wheel distance, l = 0.16

    // When using rqt_robot_steering package, the linear velocity was given to twist_.linear.x (m/s)
    // and the angular velocity was given to twist_.angular.z (rad/s)
    // wheel1 negative direction and wheel2 positive direction is foward

    // Each wheel have max velocity = 2.84 rad/s
    // It means, if the linear speed over than 

    const double wheel_distance = 0.16;
    const double wheel_radius = 0.033;
    const double wheel_circumference = wheel_radius * 2 * M_PI;

    ros::Time last_update = ros::Time::now();

    while (ros::ok())
    {
        double linear_speed = twist_.linear.x;
        double angular_speed = twist_.angular.z;

        double linear_wheel_speed = linear_speed / wheel_radius;
        double angular_wheel_speed = (wheel_distance * angular_speed) / (wheel_radius * 2);

        state.points.front().velocities[0] = linear_wheel_speed - angular_wheel_speed;
        state.points.front().velocities[1] = linear_wheel_speed + angular_wheel_speed;

        ROS_INFO_STREAM_THROTTLE(1,
            "Velocity published. " << state.points.front().velocities[0] << ", " << state.points.front().velocities[1]);        

        joint_cmd_pub_.publish(state);

        ros::spinOnce();
    }
}
