#include <ROS_Node.hpp>

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);
ros::Subscriber<std_msgs::Int16> goalSub("goal_distance", &goal_distance_cb);

geometry_msgs::Pose pose_msg;
geometry_msgs::Twist twist_msg;
std_msgs::String mechanism_states_msg;

ros::Publisher pose_pub("pose", &pose_msg);
ros::Publisher twist_pub("twist", &twist_msg);
ros::Publisher mechanism_states_pub("mechanism_state", &mechanism_states_msg);

float linear_x, angular_z;
bool is_ros_connected;
bool go_to_home = false;
int goal_distance = 0;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(goalSub);
    nh.advertise(pose_pub);
    nh.advertise(twist_pub);
    nh.advertise(mechanism_states_pub);
    nh.getHardware()->setBaud(57600);
}
void goal_distance_cb(const std_msgs::Int16 &msg)
{
    goal_distance = msg.data;
}

int get_mechanism()
{
    return goal_distance;
}

void publish_mechanism_states(int states_num)
{
    String mechanism_state[] = {
        "waiting_for_goal",
        "going_to_goal",
        "goal_reached",
        "going_to_home"};

    // mechanism_states_msg.data = mechanism_state[0].c_str();
    // mechanism_states_pub.publish(&mechanism_states_msg);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    linear_x = cmd_msg.linear.x;   // m/s
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
