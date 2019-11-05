#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/mbot_motor_command_t.hpp"

using std::cout;
using std::endl;

class Handler
{
public:
    ~Handler() {}
    
    void handleMessage(const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const lcmtypes::mbot_motor_command_t *msg)
    {
       cout << "FUCK" << endl;
    }
private:
    int num_chars_ = 0;
    int64_t start_time_ = -1;
    int64_t start_time1_ = -1;
};

int main(int argc, char **argv)
{
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=2");
    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("MBOT_MOTOR_COMMAND", &Handler::handleMessage, &handlerObject);

    while (0 == lcm.handle());

    return 0;
}