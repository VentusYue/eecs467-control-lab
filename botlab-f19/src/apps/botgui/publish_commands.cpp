#include <common/lcm_config.h>
#include <common/timestamp.h>
#include <lcmtypes/odometry_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <../lcmtypes/setpoint_t.hpp>
#include <mbot/mbot_channels.h>
#include <string>
#include <iostream>
#include <math.h>

static const double EPSILON = 0.01;
static const double TRANS_SPEED = 0.5;
static const double ANG_SPEED = M_PI / 4;

class OdoBot {
    public:
    OdoBot(lcm::LCM* lcmInstance, int argc, char** argv)
        : lcmInstance(lcmInstance) {
        odometry.x = odometry.y = odometry.theta = 0.0;
    }

    void handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odom) {  
        odometry = *odom;
    }

    void handleSetpoint(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const setpoint_t* sp) {  
        std::cout<< "Left_SP: " << sp->left_sp << ", Right_SP: " << sp->right_sp << std::endl;
    }    

    bool less_than_1() {
        return odometry.x < (1 - EPSILON) || odometry.y < (1 - EPSILON);
    }

    // bang bang control for now
    // Todo: turn it into PID control (at least proportional)
    void drive_forward() {
        mbot_motor_command_t cmd;
        cmd.left_motor_enabled = cmd.right_motor_enabled = 1;
        cmd.trans_v = TRANS_SPEED;
        cmd.angular_v = 0;
        lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    }

    void stop_motors() {
        mbot_motor_command_t cmd;
        cmd.left_motor_enabled = cmd.right_motor_enabled = 1;
        cmd.trans_v = 0;
        cmd.angular_v = 0;
        lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    }

    void turn_in_place() {
        float theta = odometry.theta;
        while ((odometry.theta - theta) < (M_PI - EPSILON)) {
            mbot_motor_command_t cmd;
            cmd.left_motor_enabled = cmd.right_motor_enabled = 1;
            cmd.trans_v = 0;
            cmd.angular_v = ANG_SPEED;
            lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    private:
    lcm::LCM* lcmInstance;                         // Instance of LCM to use for communication
    odometry_t odometry;                           // Most recent odometry measurement

};


int main(int arc, char **argv) {
    lcm::LCM lcm;
    if (!lcm.good()) return 1;
    OdoBot odo_bot = OdoBot(&lcm, arc, argv);
    
    // subscribe to odometry
    lcm.subscribe(ODOMETRY_CHANNEL, &OdoBot::handleOdometry, &odo_bot);
    lcm.subscribe("SETPOINTS", &OdoBot::handleSetpoint, &odo_bot);

    // loop
    while (true) {
        // while less than 1 m, forward
        // while (odo_bot.less_than_1()) {
        //     // publish motor command (setpoints)
        //     odo_bot.drive_forward();
        // }
        // odo_bot.stop_motors();
        // // else turn in place counterclockwise 90 degrees
        // odo_bot.turn_in_place();
        // odo_bot.stop_motors();

        odo_bot.drive_forward();
    }

    return 0;
}
