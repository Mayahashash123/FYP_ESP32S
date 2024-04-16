#include <mechanism.hpp>
#include "ROS_Node.hpp"
float goal_steps = 0;
int encoder, prev_encoder, total_counter = 0, prev_limit;
volatile int current_steps = 0;

double distance = 0, serial_input;
bool current_LS = 1;
int state = 0;
unsigned long previous_counting_millis;
// bool goal = false;
mechanism_state Mechanism_State;
// TaskHandle_t encoderTask;
// void Task1code(void *parameter);

void mechanism_init()
{
    pinMode(limit_switch, INPUT_PULLUP);
    pinMode(encoder_pin, INPUT_PULLDOWN);
    pinMode(hall_sensor, INPUT);
    pinMode(motor_up, OUTPUT);
    pinMode(motor_down, OUTPUT);
    pinMode(laser_pin, OUTPUT);
    prev_encoder = digitalRead(encoder_pin);
    prev_limit = digitalRead(limit_switch);
    digitalWrite(laser_pin, LOW);

    Mechanism_State = going_to_home;
    current_steps = 0;
    Serial.println("mechanism setup");
}

int move_mechanism(float distance)
{
    bool go_home = false;

    if (distance == 0 && Mechanism_State == waiting_for_goal)
        return Mechanism_State;

    else if (distance == 0 && Mechanism_State == goal_reached)
        go_home = true;

    prev_encoder = encoder;
    encoder = digitalRead(encoder_pin);
    if (Mechanism_State == waiting_for_goal)
    {
        goal_steps = round((44.0 * (2.5 + (distance - 25.0))) / 153.0); //
        // print_on_terminal("goal_steps: ", goal_steps);
        Mechanism_State = going_to_goal;
    }
    else if (Mechanism_State == going_to_goal)
    {
        // print_on_terminal("encoder val", encoder);
        // print_on_terminal("encoder prev val", prev_encoder);

        if (current_steps < goal_steps)
        {
            // goal = true;
            move_up();

            if (encoder && !prev_encoder)
            {
                current_steps++;
                previous_counting_millis = millis();
            }
            Serial.println("current_steps: " + String(current_steps));
            print_on_terminal("encoder: ", encoder);
            print_on_terminal("prev_encoder: ", prev_encoder);
            print_on_terminal("going to maya steps: ", current_steps);
            print_on_terminal("-------------------");
        }
        else
        {
            // goal = false;
            stop();
            Mechanism_State = goal_reached;
        }
        if (analogRead(hall_sensor) < 30 || current_steps > 44)
        {
            // goal = false;
            stop();
            encoder = !encoder;
            Mechanism_State = goal_reached;
        }
    }
    else if (Mechanism_State == goal_reached)
    {
        if (go_home)
        {
            Mechanism_State = going_to_home;
        }
    }
    else if (Mechanism_State == going_to_home)
    {
        move_down();
        prev_limit = current_LS;
        print_on_terminal("going to home steps: ", current_steps);
        print_on_terminal("-------------------");
        current_LS = digitalRead(limit_switch);
        if (!current_LS && prev_limit) // going back home
        {
            stop();
            Mechanism_State = waiting_for_goal;
            current_steps = 0;
            goal_steps = 0;
        }
    }

    return Mechanism_State;
}

void move_up()
{
    analogWrite(motor_up, 200);
    analogWrite(motor_down, 0);
}

void move_down()
{
    analogWrite(motor_up, 0);
    analogWrite(motor_down, 200);
}

void stop()
{
    analogWrite(motor_up, 0);
    analogWrite(motor_down, 0);
    // encoder = !encoder;
}