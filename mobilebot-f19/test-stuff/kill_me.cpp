#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/mbot_motor_command_t.hpp"
#include <unistd.h>

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    lcmtypes::mbot_motor_command_t my_data;
    my_data.left_motor_enabled = 1;
    my_data.left_motor_speed = 0.5;
    lcm.publish("MBOT_MOTOR_COMMAND", &my_data);
    usleep(5 * 1E9);
    my_data.left_motor_speed = 0;
    lcm.publish("MBOT_MOTOR_COMMAND", &my_data);

    return 0;
}