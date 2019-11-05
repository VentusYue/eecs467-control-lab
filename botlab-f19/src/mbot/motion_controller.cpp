#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/setpoint_t.hpp>
#include <lcmtypes/turn_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <math.h>
#include <fstream>
#include <iomanip>

static const float EPSILON = 0.02;
static const float TRANS_SPEED = 0.5;
static const float ANG_SPEED = 0.5 * M_PI;
// static const float K1 = sqrt(8);
// static const float K2 = 2;

static const float K1 = sqrt(2);
static const float K2 = 0.5;

int current_state = 0;

class MotionController
{
public:

    // std::ofstream sp_data; // TONY: For writing to csv

    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance) : lcmInstance(instance)
    {
        time_offset = 0;
        timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";
    }

    /**
    * updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    *
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        // TODO(EECS467): Implement your feedback controller for Task V and VI here.
        mbot_motor_command_t command;
        command.trans_v = 0.0f;
        command.angular_v = 0.0f;
        command.utime = now();

        //if(targets_.empty()) return cmd; // TONY TODO: NO PATH JUST ODOMETRY FOR NOW
        float error = 0.0;
        float target_theta = 0.0;
        float error_theta = 0.0;
        if (current_state >= 4) {
            current_state -= 4;
        }
        switch (current_state)
        {
        case 0:
            error = odometry_y;
            target_theta = 0;
            break;
        case 1:
            error = odometry_x - 1;
            target_theta =  M_PI / 2;
            break;
        case 2:
            error = odometry_y - 1;
            target_theta =  M_PI;
            break;
        case 3:
            error = odometry_x;
            target_theta =  3 * M_PI / 2;
            break;
        }

        error_theta_prev = error_theta;

        error_theta =  target_theta - odometry_theta;
        if (error_theta - error_theta_prev >  1.5 * M_PI) {
            error_theta -= 2 * M_PI;
        } else if (error_theta - error_theta_prev < - 1.5 * M_PI) {
            error_theta += 2 * M_PI;
        }
        std::cout  << " state: "<< current_state << " error_theta: " << std::setprecision(3) << error_theta
        << " current theta: " << std::setprecision(3) << odometry_theta << " error: " << std::setprecision(3) << error << std::endl;
        if(state_ == TURN) {
            // TODO(EECS467): Implement your feedback controller for turning here.
            // std::cout << "TURNING\n";
            command.trans_v = 0;
            command.angular_v = ANG_SPEED;

        } else if(state_ == DRIVE) {
            // TODO(EECS467): Implement your feedback controller for driving here.
            command.trans_v = TRANS_SPEED;
            //std::cout << "TRANS VELOCITY : " << command.trans_v << std::endl;
            // update angular velocity
            // w = -k1 * theta - k2/v0 * e
            command.angular_v =  K1 * error_theta + K2 / TRANS_SPEED * error;
        } else if (state_ == STOP) {
            command.trans_v = 0;
            command.angular_v = 0;
        } else {
            std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
        }

        return command;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync){
    	timesync_initialized_ = true;
    	time_offset = timesync->utime-utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime << "\n";
    	for(auto pose : targets_){
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << "\n";

        confirm.utime = now();
        confirm.creation_time = path->utime;
        confirm.channel = channel;

        //confirm that the path was received
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        odometry_x = odometry->x, odometry_y = odometry->y, odometry_theta = odometry->theta;
        // TODO(EECS467) Implement your handler for new odometry data
        // std::cout << odometry->x << " " << odometry->y << std::endl;
        if (current_state >= 4) {
            current_state -= 4;
            // state_ = STOP;
        }
        if (state_ == DRIVE) {
            bool start_turn = false;
            switch (current_state)
            {
            case 0:
                if ( odometry->x > 1.0 - EPSILON )
                {
                    // std::cout << "case 0" << std::endl;
                    start_turn = true;
                }
                break;
            case 1:
                if ( odometry->y > 1.0 - EPSILON )
                {
                    // std::cout << "case 1" << std::endl;
                    start_turn = true;
                }
                break;
            case 2:
                if ( odometry->x < EPSILON )
                {
                    // std::cout << "case 2" << std::endl;
                    start_turn = true;
                }
                break;
            case 3:
                if ( odometry->y < EPSILON )
                {
                    // std::cout << "case 3" << std::endl;
                    start_turn = true;
                }
                break;
            }
            if ( start_turn == true ) {
                std::cout << "Start turning: " << std::endl;
                state_ = TURN;
                ++current_state;
                theta = odometry->theta;
                // todo: send a turn_t to botgui
                turn_t turn;
                turn.x = odometry_x;
                turn.y = odometry_y;
                std::cout << "Red square at: " << turn.x << ", " << turn.y << std::endl;
                lcmInstance->publish("TURN CHANNEL", &turn);
                // std::cout << "need to turn because (x, y) = " << odometry->x << " " << odometry->y << std::endl;
                // todo: reset error integral term

                // temporary, make it stop
                //state_ = STOP;
                // odo_x = odometry->x;
                // odo_y = odometry->y;
            }
        } else {
            float dtheta = odometry->theta - theta;

            // std::cout << "DTHETA: " << abs(dtheta) << std::endl;
            if ((dtheta > (M_PI / 2 - 0.15) || dtheta < (-M_PI / 2 + 0.15) ) 
            && fabs(dtheta) < 1.6 * M_PI ) {
                state_ = DRIVE;
                std::cout << "Start driving: " << dtheta << std::endl;
                //++current_state; don't think this should be here - jonas and yue
                // odo_x = odometry->x;
                // odo_y = odometry->y;
            }
        }
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        // TODO(EECS467) Implement your handler for new pose data (from the laser scan)
    }

    void handleSetpoint(const lcm::ReceiveBuffer* buf, const std::string& channel, const setpoint_t* sp) {
        // TONY: saves current left right velocities and time to a CSV
        int64_t    utime;
        float      left_sp;
        float      right_sp;
        // sp_data << sp->utime << "," << sp->left_sp << "," << sp->right_sp << "\n";
    }

private:

    enum State
    {
        TURN,
        DRIVE,
        STOP
    };

    std::vector<pose_xyt_t> targets_;
    // TODO(EECS467) Initialize the state.
    State state_ = DRIVE;
    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    float theta;
    float odo_x = 0, odo_y = 0;
    float odometry_x, odometry_y, odometry_theta;
    int64_t time_offset;
    bool timesync_initialized_;

    float error_theta_prev; // for checking if the fxxking clamp function is used by odometry

    message_received_t confirm;
    lcm::LCM * lcmInstance;

    int64_t now(){
        return utime_now()+time_offset;
    }
};


int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);

    MotionController controller(&lcmInstance);
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, &controller);
    // TODO(EECS467) Add additional subscribers to your customized messages.
    // For instance, instantaneous translational and rotational velocity of the robot, which is necessary
    // for your feedback controller.
    // lcmInstance.subscribe("SETPOINTS", &MotionController::handleSetpoint, &controller);

    // controller.sp_data.open("task3.csv");
    //controller.sp_data << "Timestamp,Left_sp,Right_sp\n";

    signal(SIGINT, exit);
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
        //if(controller.timesync_initialized()){
            mbot_motor_command_t cmd = controller.updateCommand();
            // cmd.left_motor_speed = -TRANS_SPEED; // TONY: FOR TASK 3
            // cmd.right_motor_speed = TRANS_SPEED;
            // std::cout << "Updated motor command\n";
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        //}
    }

    //controller.sp_data.close();

    return 0;
}
