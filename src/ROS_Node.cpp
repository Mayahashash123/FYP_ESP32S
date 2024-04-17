#include <ROS_Node.hpp>

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);
ros::Subscriber<std_msgs::Int16> goalSub("goal_distance", &goal_distance_cb);
ros::Subscriber<std_msgs::Bool> laserSub("laser_pointer", &laser_pointer_cb);

geometry_msgs::Pose pose_msg;
geometry_msgs::Twist twist_msg;
std_msgs::String mechanism_states_msg;
std_msgs::Float32 battery_level;

ros::Publisher pose_pub("pose", &pose_msg);
ros::Publisher twist_pub("twist", &twist_msg);
ros::Publisher mechanism_states_pub("mechanism_state", &mechanism_states_msg);
ros::Publisher batteryPub("battery_percentage", &battery_level);

float linear_x, angular_z;
bool is_ros_connected;
bool go_to_home = false;
int goal_distance = 0;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(goalSub);
    nh.subscribe(laserSub);

    nh.advertise(pose_pub);
    nh.advertise(twist_pub);
    nh.advertise(mechanism_states_pub);
    nh.advertise(batteryPub);

    nh.getHardware()->setBaud(57600);
}
void goal_distance_cb(const std_msgs::Int16 &msg)
{
    goal_distance = msg.data;
    print_on_terminal("goal distance: ", float(goal_distance));
}

int get_mechanism()
{
    // print_on_terminal("goal_distance:::: ", goal_distance);
    return goal_distance;
}

void publish_mechanism_states(int states_num)
{
    String mechanism_state[] = {
        "waiting_for_goal",
        "going_to_goal",
        "goal_reached",
        "going_to_home"};

    Serial.println(mechanism_state[states_num]);
    mechanism_states_msg.data = mechanism_state[states_num].c_str();
    mechanism_states_pub.publish(&mechanism_states_msg);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    linear_x = cmd_msg.linear.x;   // m/s
    angular_z = cmd_msg.angular.z; // rad/s
    nh.loginfo((" " + String(linear_x)).c_str());
}

void laser_pointer_cb(const std_msgs::Bool &pointer_msg)
{
    digitalWrite(laser_pin, pointer_msg.data);
    print_on_terminal("laser data: ", pointer_msg.data);
}

void publish_odom(std::pair<geometry_msgs::Pose, geometry_msgs::Twist> odometry)
// void publish_odom()
{
    // print_on_terminal("publish odom function");
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

void publish_battery_percentage(float battery_percentage)
{
    battery_level.data = battery_percentage;
    batteryPub.publish(&battery_level);
}

float get_linear_velocity()
{
    // nh.loginfo(("linear: " + String(linear_x)).c_str());
    return linear_x;
}

float get_angular_velocity()
{
    // nh.loginfo(("angular: " + String(angular_z)).c_str());
    // nh.loginfo("-------------------");
    return angular_z;
}

void print_on_terminal(String msg, float value)
{
    nh.loginfo((msg + " " + String(value)).c_str());
}

void ROS_Update()
{
    is_ros_connected = nh.connected();
    // digitalWrite(2, is_ros_connected);
    nh.spinOnce();
    delay(1);
}

ros::NodeHandle get_nh()
{
    return nh;
}