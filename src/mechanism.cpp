#include <mechanism.hpp>

int goal_steps = 0;
int encoder, prev_encoder, total_counter = 0, current_steps = 0, prev_limit;
double distance = 0, serial_input;
bool current_LS;
int state = 0;

mechanism_state Mechanism_State;
void mechanism_init()
{
    pinMode(limit_switch, INPUT_PULLUP);
    pinMode(encoder_pin, INPUT);
    pinMode(hall_sensor, INPUT);
    prev_encoder = digitalRead(encoder_pin);
    prev_limit = digitalRead(limit_switch);

    pinMode(motor_up, OUTPUT);
    pinMode(motor_down, OUTPUT);
    Serial.println("mechanism init");

    while (digitalRead(limit_switch)) // homming
    {
        analogWrite(motor_up, 0);
        analogWrite(motor_down, 150);
    }
    analogWrite(motor_up, 0);
    analogWrite(motor_down, 0);
    Mechanism_State = waiting_for_goal;
    current_steps = 0;
}

mechanism_state move_mechanism(float distance, bool go_home)
{
    if (distance == 0)
        return Mechanism_State;

    prev_encoder = encoder;
    encoder = digitalRead(encoder_pin);

    if (Mechanism_State == waiting_for_goal)
    {
        goal_steps = (44 * (2.5 + (distance - 25))) / 150;
        Mechanism_State = going_to_goal;
    }
    else if (Mechanism_State == going_to_goal)
    {
        if (current_steps < goal_steps)
        {
            move_up();
            if (encoder && !prev_encoder)
            {
                current_steps++;
            }
        }
        else
        {
            Mechanism_State = goal_reached;
        }
    }
    else if (Mechanism_State == goal_reached)
    {
        stop();
        if (go_home)
        {
            Mechanism_State = going_to_home;
        }
    }
    else if (Mechanism_State == going_to_home)
    {
        move_down();
        prev_limit = current_LS;

        current_LS = digitalRead(limit_switch);
        if (!current_LS && prev_limit) // going back home
        {
            stop();
            Mechanism_State = waiting_for_goal;
            current_steps = 0;
            goal_steps = 0;
        }
    }

    if (digitalRead(hall_sensor) < 30 || total_counter > 44)
    {
        stop();
    }
    return Mechanism_State;
}

void move_up()
{
    analogWrite(motor_up, 140);
    analogWrite(motor_down, 0);
}

void move_down()
{
    analogWrite(motor_up, 0);
    analogWrite(motor_down, 160);
}

void stop()
{
    analogWrite(motor_up, 0);
    analogWrite(motor_down, 0);
    encoder = !encoder;
}