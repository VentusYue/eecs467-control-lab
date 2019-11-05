#include "lcmtypes/mbot_motor_command_t.hpp"
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
    lcm::LCM lcm;

    for (int i = 0; i < 4; ++i)
    {
        cout << "Moving Forward" << endl;
        mbot_motor_command_t msg;
        msg.left_motor_speed = SPEED;
        msg.right_motor_speed = SPEED;
        lcm.publish(MBOT_MOTOR_COMMAND_CHANNEL, &msg);
        usleep(1e6 * DIST_SIDE / SPEED);

        cout << "Turning Left" << endl;
        msg.left_motor_speed = -SPEED;
        msg.right_motor_speed = SPEED;
        lcm.publish(MBOT_MOTOR_COMMAND_CHANNEL, &msg);
        usleep(1e6 * WHEEL_BASE * M_PI / 4 / SPEED);
    }

    return 0;
}
