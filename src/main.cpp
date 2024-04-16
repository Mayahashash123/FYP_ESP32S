#include <Arduino.h>
#include <RC_Control.hpp>
#include <ODrive.hpp>
#include <ROS_Node.hpp>
#include <mechanism.hpp>

unsigned long previous_millis, time_interval = 100;
unsigned long ros_t, prev_ros = 0;

float linear = 0, angular = 0;
int mechanism_rc_goal;
int goalDistance, val;
void setup()
{
  Serial.begin(57600);
  mechanism_init();
  bluetooth_init();
  Odrive_init();
  Rosserial_init();
  pinMode(2, OUTPUT);
}
bool prev_enc, enc;

std::pair<float, float> RC_Cmd(0, 0);
std::pair<float, float> Ros_Cmd;

void loop()
{

  //?RC control
  RC_Control(RC_Cmd.first, RC_Cmd.second, mechanism_rc_goal); // zeros if not RC connected

  //?cmd_vel control
  if (!is_ros_connected && !is_bluetooth_connected)
  {
    linear = angular = mechanism_rc_goal = 0;
  }
  else
  {

    if (!is_ros_connected) // RC but no ROS
    {
      linear = RC_Cmd.first;
      angular = RC_Cmd.second;
      // goalDistance = int(mechanism_rc_goal);
    }
    else // ROS connected
    {
      Ros_Cmd.first = get_linear_velocity();
      Ros_Cmd.second = get_angular_velocity();

      if (Ros_Cmd.first == 0 && Ros_Cmd.second == 0)
      { // in case bt is not connected RC_Cmd is 0 and in case it is connected it will take the last value of RC_Cmd which it should be for the first time 0
        linear = RC_Cmd.first;
        angular = RC_Cmd.second;
      }
      else
      {
        linear = Ros_Cmd.first;
        angular = Ros_Cmd.second;
      }

      //?publishing odometry feedback
      if (millis() - previous_millis > time_interval)
      {
        //?Battery Level
        publish_battery_percentage(getvoltage());

        // ros_t = millis();
        publish_odom(getOdom());
        // print_on_terminal(String(millis()-ros_t));
        previous_millis = millis();
      }
    }
  }

  // print_on_terminal("main loop core: ", xPortGetCoreID());

  //?driving motors based on either RC control or cmd_vel
  driveMotors(linear, angular);

  // ? Mechanism control
  goalDistance = get_mechanism();
  val = move_mechanism(goalDistance);
  publish_mechanism_states(val);

  //?Laser_Pointer control

  // ros_spin
  ROS_Update();
}
