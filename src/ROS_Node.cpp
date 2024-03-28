#include <ROS_Node.hpp>

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);
geometry_msgs::Pose pose_msg;
geometry_msgs::Twist twist_msg;
ros::Publisher pose_pub("pose", &pose_msg);
ros::Publisher twist_pub("twist", &twist_msg);

float linear_x, angular_z;
bool is_ros_connected;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pose_pub);
    nh.advertise(twist_pub);
    nh.getHardware()->setBaud(57600);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    linear_x = cmd_msg.linear.x;       // m/s
    angular_z = cmd_msg.angular.z; // rad/s
    // nh.loginfo((" " + String(linear_x)).c_str());
}

void publish_odom(std::pair<geometry_msgs::Pose, geometry_msgs::Twist> odometry)
// void publish_odom()
{
    const geometry_msgs::Pose input_pose_msg = odometry.first;
    const geometry_msgs::Twist input_twist_msg = odometry.second;

    pose_msg.position.x = input_pose_msg.position.x;
    pose_msg.position.y = input_pose_msg.position.y;
    pose_msg.orientation.z = input_pose_msg.orientation.z;
    pose_msg.orientation.w = input_pose_msg.orientation.w;

    twist_msg.linear.x = input_twist_msg.linear.x;
    twist_msg.angular.z = input_twist_msg.angular.z;

    // Publish the odom message
    pose_pub.publish(&pose_msg);
    twist_pub.publish(&twist_msg);
}

float get_linear_velocity()
{
    nh.loginfo(("linear: " + String(linear_x)).c_str());
    return linear_x;
}

float get_angular_velocity()
{
    nh.loginfo(("angular: " + String(angular_z)).c_str());
    nh.loginfo("-------------------");
    return angular_z;
}

void print_on_terminal(String msg, float value)
{
    nh.loginfo((msg + " " + String(value)).c_str());
}

void ROS_Update()
{
    is_ros_connected = nh.connected();
    digitalWrite(2, is_ros_connected);
    nh.spinOnce();
    delay(1);
}
