#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/setpoint_t.hpp"

using std::cout;
using std::endl;

class Handler
{
public:
    ~Handler() {}

    void handleMessage(const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const lcmtypes::setpoint_t *msg)
    {
        cout << msg->utime << ',' << msg->left_sp << ',' << msg->right_sp << ','
        << msg->left_vel << ',' << msg->right_vel << endl;
    }

private:
    int num_chars_ = 0;
    int64_t start_time_ = -1;
    int64_t start_time1_ = -1;
};

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("SETPOINTS", &Handler::handleMessage, &handlerObject);

    while (0 == lcm.handle())
        ;

    return 0;
}