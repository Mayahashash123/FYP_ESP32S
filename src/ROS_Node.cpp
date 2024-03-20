#include <ROS_Node.hpp>

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped transform_msg;
tf::TransformBroadcaster broadcaster;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);
ros::Publisher odom_pub("odom", &odom_msg);

float linear_x, angular_z;
bool is_ros_connected;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(odom_pub);
    nh.getHardware()->setBaud(57600);
    broadcaster.init(nh);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    linear_x = cmd_msg.linear.x,       // m/s
        angular_z = cmd_msg.angular.z; // rad/s
    // nh.loginfo((" " + String(linear_x)).c_str());
}


// void publish_odom(const nav_msgs::Odometry odom)
// {
//     odom_msg.header.stamp = nh.now();
//     odom_msg.child_frame_id = "base_footprint";
//     odom_msg.header.frame_id = "odom";
        
//     odom_msg.pose.pose.position.x = odom.pose.pose.position.x;
//     odom_msg.pose.pose.position.y = odom.pose.pose.position.y;
//     odom_msg.pose.pose.position.z = 0;

//     odom_msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
//     odom_msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;
//     odom_msg.pose.pose.orientation.x = 0;
//     odom_msg.pose.pose.orientation.y = 0;

//     // Publish the odom message
//     odom_pub.publish(&odom_msg);

//     transform_msg.child_frame_id = "base_footprint";
//     transform_msg.header.frame_id = "odom";
//     transform_msg.header.stamp = nh.now();

//     transform_msg.transform.translation.x = odom_msg.pose.pose.position.x;
//     transform_msg.transform.translation.y = odom_msg.pose.pose.position.y ;
//     transform_msg.transform.translation.z = 0;

//     transform_msg.transform.rotation.x = 0;
//     transform_msg.transform.rotation.y = 0;
//     transform_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
//     transform_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w;

//     broadcaster.sendTransform(transform_msg);
// }

float get_linear_velocity()
{
    return linear_x;
}

float get_angular_velocity()
{
    return angular_z;
}

void print_on_terminal(String msg, float value)
{
    nh.loginfo((msg+ " " + String(value)).c_str());
}

void publish_odom()
{
    odom_msg.header.stamp = nh.now();
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.header.frame_id = "odom";
        
    odom_msg.pose.pose.position.x = 0.12;
    odom_msg.pose.pose.position.y = 0.14;
    odom_msg.pose.pose.position.z = 0;

    odom_msg.pose.pose.orientation.z = 0.15;
    odom_msg.pose.pose.orientation.w = 0.16;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;

    // Publish the odom message
    odom_pub.publish(&odom_msg);

    transform_msg.child_frame_id = "base_footprint";
    transform_msg.header.frame_id = "odom";
    transform_msg.header.stamp = nh.now();

    transform_msg.transform.translation.x = 0.5;
    transform_msg.transform.translation.y = 0.3 ;
    transform_msg.transform.translation.z = 0;

    transform_msg.transform.rotation.x = 0;
    transform_msg.transform.rotation.y = 0;
    transform_msg.transform.rotation.z = 0.2;
    transform_msg.transform.rotation.w = 0.4;

    broadcaster.sendTransform(transform_msg);
}

void ROS_Update()
{
    digitalWrite(2, nh.connected());
    is_ros_connected = nh.connected();
    nh.spinOnce();
    delay(1);
}
