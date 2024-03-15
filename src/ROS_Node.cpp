#include <ROS_Node.hpp>

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
geometry_msgs::TransformStamped transform_msg;
tf::TransformBroadcaster broadcaster;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);
ros::Publisher odom_pub("odom", &odom_msg);

float linear_x, angular_z;

void Rosserial_init()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(odom_pub);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_msg)
{
    linear_x = cmd_msg.linear.x,       // m/s
        angular_z = cmd_msg.angular.z; // rad/s
    nh.loginfo((" " + String(linear_x)).c_str());
}

void publish_odom(const nav_msgs::Odometry odom)
{
    odom_msg.pose.pose.position.x = odom.pose.pose.position.x;
    odom_msg.pose.pose.position.y = odom.pose.pose.position.y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    odom_msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;

    // Publish the odom message
    odom_pub.publish(&odom_msg);

    transform_msg.child_frame_id = "base_footprint";
    transform_msg.header.frame_id = "odom";
    transform_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    transform_msg.transform.translation.y = odom_msg.pose.pose.position.y ;
    transform_msg.transform.translation.z = 0;

    transform_msg.transform.rotation.x = 0;
    transform_msg.transform.rotation.y = 0;
    transform_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    transform_msg.transform.rotation.x = odom_msg.pose.pose.orientation.w; 
    broadcaster.sendTransform(transform_msg);
}

float get_linear_velocity()
{
    return linear_x;
}

float get_angular_velocity()
{
    return angular_z;
}

void print_on_terminal(float msg)
{
    nh.loginfo((" " + String(msg)).c_str());
}

void ROS_Update()
{
    nh.spinOnce();
    delay(1);
}
