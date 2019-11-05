#include "../lcmtypes/mbot_motor_command_t.hpp"
#include "../lcmtypes/turn_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "../common/mb_defs.h"

#define SPEED 0.25
#define DIST_SIDE 1

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    cout << "Commanding robot to drive around 1m square" << endl;
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=2");

    mbot_motor_command_t msg;
    for (int i = 0; i < 12; ++i)
    {
        cout << "Moving Forward" << endl;
        msg.left_motor_speed = SPEED;
        msg.right_motor_speed = SPEED;
        lcm.publish(MBOT_MOTOR_COMMAND_CHANNEL, &msg);
        usleep(1e6 * DIST_SIDE / SPEED);

        cout << "Turning Left" << endl;
        turn_t turn;
        turn.x = -1;
        turn.y = -1;
        lcm.publish("TURN CHANNEL", &turn);
        msg.left_motor_speed = -SPEED;
        msg.right_motor_speed = SPEED;
        lcm.publish(MBOT_MOTOR_COMMAND_CHANNEL, &msg);
        usleep(1e6 * WHEEL_BASE * M_PI / 4 / SPEED);
    }
    msg.left_motor_speed = 0;
    msg.right_motor_speed = 0;
    lcm.publish(MBOT_MOTOR_COMMAND_CHANNEL, &msg);

    return 0;
}
