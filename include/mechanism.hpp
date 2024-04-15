#ifndef _MECHANISM_HPP
#define _MECHANISM_HPP

#include <Arduino.h>
#include <analogWrite.h>

#define limit_switch 27
#define motor_up 33
#define motor_down 32
#define encoder_pin 12
#define hall_sensor 14

 enum mechanism_state
{
    waiting_for_goal,
    going_to_goal,
    goal_reached,
    going_to_home
}; 
void mechanism_init();
int move_mechanism(float distance);

void move_down();
void move_up();
void stop();



#endif