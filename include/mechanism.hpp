#ifndef _MECHANISM_HPP
#define _MECHANISM_HPP

#include <Arduino.h>

#define limit_switch 27
#define motor_up 33
#define motor_down 32
#define encoder_pin 25
#define hall_sensor 26

 enum mechanism_state
{
    waiting_for_goal,
    going_to_goal,
    goal_reached,
    going_to_home
}; 
void mechanism_init();
mechanism_state move_mechanism(float distance, bool go_home);
void move_down();
void move_up();
void stop();



#endif