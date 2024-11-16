#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

ros::Publisher joint_state_pub_;
ros::Publisher joint_cmd_pub_;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

geometry_msgs::Pose z_rotation_pose_;
geometry_msgs::Pose target_pose_;

std::vector<double> arm_target;


inline double get_center_angle_using_three_length(double a, double b, double c)
{
    return std::acos((a * a + b * b - c * c) / (2 * a * b));
}

Eigen::Matrix4d pose_to_marix(geometry_msgs::Pose pose)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
    matrix(0, 3) = pose.position.x;
    matrix(1, 3) = pose.position.y;
    matrix(2, 3) = pose.position.z;
    return matrix;
}

void solve_robot_arm_ik()
{
    Eigen::Vector3d link3_to_4(0.024, 0, 0.128);

    const double d1 = 0.0595;
    const double link3_length = std::sqrt(0.024 * 0.024 + 0.128 * 0.128);
    const double link4_length = 0.124;
    const double link5_length = 0.0817;

    const double link3_angle = std::atan2(0.024, 0.128);
    std::cout << "link3_angle: " << link3_angle << std::endl;

    Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
    target_transform.block<3, 3>(0, 0) = pose_to_marix(z_rotation_pose_).block<3, 3>(0, 0);
    target_transform = target_transform * pose_to_marix(target_pose_);

    std::cout << target_transform << std::endl;

    Eigen::Vector3d wrist_position = target_transform.block<3, 1>(0, 3) - target_transform.block<3, 1>(0, 2) * link5_length;
    std::cout << wrist_position << std::endl;

    double x_prime = Eigen::Vector2d(wrist_position.block<2, 1>(0, 0)).norm();
    Eigen::Vector2d m_vec = Eigen::Vector2d(x_prime, wrist_position.z() - d1);
    double m = m_vec.norm();
    double alpha = std::atan2(m_vec.y(), m_vec.x());

    double gamma = get_center_angle_using_three_length(link3_length, link4_length, m);
    double beta  = get_center_angle_using_three_length(m, link3_length, link4_length);


    if (gamma > 0.0) gamma -= M_PI;
    if (beta < 0.0) beta += M_PI;

    double q1 = std::atan2(wrist_position.y(), wrist_position.x());
    double q2 = M_PI / 2 - alpha - beta - link3_angle;
    double q3 = -(gamma - link3_angle + M_PI / 2);
    Eigen::Matrix4d link4_position = Eigen::Affine3d(
        Eigen::Translation3d(0, 0, d1) *
        Eigen::AngleAxisd(q1, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(q2 + link3_angle, Eigen::Vector3d::UnitY()) *
        Eigen::Translation3d(0, 0, link3_length) *
        Eigen::AngleAxisd(q3 - link3_angle, Eigen::Vector3d::UnitY()) *
        Eigen::Translation3d(0, 0, link4_length)).matrix();

    Eigen::Matrix3d orientation = link4_position.block<3, 3>(0, 0).transpose() * target_transform.block<3, 3>(0, 0);
    std::cout << orientation << std::endl;

    double distance_from_linke3_to_target = 0.01;
    double q4 = -(M_PI - get_center_angle_using_three_length(link4_length, link5_length, distance_from_linke3_to_target));
    q4 = std::atan2(orientation(0, 2), orientation(0, 0)) - M_PI / 2;

    std::cout << "Solution : " << q1 << ", " << q2 << ", " << q3 << ", " << q4 << std::endl;

    std::vector<double> solution;
    solution.push_back(q1);
    solution.push_back(q2);
    solution.push_back(q3);
    solution.push_back(q4);
    arm_target = solution;
}

Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.05;
    marker.scale.y = msg.scale * 0.05;
    marker.scale.z = msg.scale * 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

Marker makeArrow( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::ARROW;
    marker.scale.x = msg.scale * 0.05;
    marker.scale.y = msg.scale * 0.1;
    marker.scale.z = msg.scale * 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);

    point.z = 0.15;
    marker.points.push_back(point);

    return marker;
}

InteractiveMarkerControl& makeArrowControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeArrow(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                    << ", " << feedback->mouse_point.y
                    << ", " << feedback->mouse_point.z
                    << " in frame " << feedback->header.frame_id;
    }

    if (feedback->marker_name == "rotation_z")
    {
        z_rotation_pose_.orientation = feedback->pose.orientation;
    }
    else if (feedback->marker_name == "target")
    {
        target_pose_ = feedback->pose;
    }
    
    solve_robot_arm_ik();

    server->applyChanges();

    
}

void makeZRotationMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "arm_base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.2;

    int_marker.name = "rotation_z";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    control.orientation.w = 0.70711;
    control.orientation.x = 0;
    control.orientation.y = 0.70711;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeTargetMarker( const tf::Vector3& position )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "target_ref";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.2;

    int_marker.name = "target";

    makeArrowControl(int_marker);

    InteractiveMarkerControl control;
    control.orientation_mode = InteractiveMarkerControl::FIXED;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    // control.name = "move_y";
    // control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    // int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.012, 0.0, 0.017));
  t.setRotation(tf::Quaternion(z_rotation_pose_.orientation.x, z_rotation_pose_.orientation.y, z_rotation_pose_.orientation.z, z_rotation_pose_.orientation.w));
  br.sendTransform(tf::StampedTransform(t, time, "arm_base_link", "target_ref"));

  counter++;
}

template <typename Type>
inline int get_item_index(std::vector<Type> array, Type value)
{
    auto iter = std::find(array.begin(), array.end(), value);
    if (iter == array.end()) return -1;
    return static_cast<int>(std::distance(array.begin(), iter));
}


void joint_state_callback(const sensor_msgs::JointState::ConstPtr& state)
{
    if (arm_target.size() == 0) return;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.header.frame_id = "base";
    joint_trajectory.joint_names.push_back("Arm1");
    joint_trajectory.joint_names.push_back("Arm2");
    joint_trajectory.joint_names.push_back("Arm3");
    joint_trajectory.joint_names.push_back("Arm4");
    joint_trajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    // joint_trajectory.points.front().velocities.push_back((state->position.at(get_item_index<std::string>(state->name, "Arm1")) - arm_target.at(0)) * 10.0);
    // joint_trajectory.points.front().velocities.push_back((state->position.at(get_item_index<std::string>(state->name, "Arm2")) - arm_target.at(1)) * 10.0);
    // joint_trajectory.points.front().velocities.push_back((state->position.at(get_item_index<std::string>(state->name, "Arm3")) - arm_target.at(2)) * 10.0);
    // joint_trajectory.points.front().velocities.push_back((state->position.at(get_item_index<std::string>(state->name, "Arm4")) - arm_target.at(3)) * 10.0);
    joint_trajectory.points.front().positions.push_back(arm_target.at(0));
    joint_trajectory.points.front().positions.push_back(arm_target.at(1));
    joint_trajectory.points.front().positions.push_back(arm_target.at(2));
    joint_trajectory.points.front().positions.push_back(arm_target.at(3));

    std::cout << "Joint velocity : [ " << 
        joint_trajectory.points.front().positions.at(0) << ", " <<
        joint_trajectory.points.front().positions.at(1) << ", " <<
        joint_trajectory.points.front().positions.at(2) << ", " <<
        joint_trajectory.points.front().positions.at(3) << "]" << std::endl;

    joint_cmd_pub_.publish(joint_trajectory);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_arm_drive");
    ros::NodeHandle nh;

    z_rotation_pose_.position.x = 0.012;
    z_rotation_pose_.position.y = 0;
    z_rotation_pose_.position.z = 0.017;
    z_rotation_pose_.orientation.w = 1;
    z_rotation_pose_.orientation.x = 0;
    z_rotation_pose_.orientation.y = 0;
    z_rotation_pose_.orientation.z = 0;
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);
    server.reset( new interactive_markers::InteractiveMarkerServer("robot_arm_drive","",false) );

    joint_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_cmd", 10);
    ros::Subscriber joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, joint_state_callback);


    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    
    tf::Vector3 position;
    position = tf::Vector3(0, 0, 0);
    makeZRotationMarker(position);
    makeTargetMarker(position);

    server->applyChanges();

    ros::spin();

    server.reset();
}