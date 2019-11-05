#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/setpoint_t.hpp>
#include <lcmtypes/lidar_t.hpp>
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

static const float EPSILON = 0.01;
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
        float error_theta = M_PI / 2 - theta_straight;
        float error = initial_lidar_dist - distance_from_wall;

        // std::cout  << " state: "<< current_state << " error_theta: " << std::setprecision(3) << error_theta
        // << " current theta: " << std::setprecision(3) << odometry_theta << " error: " << std::setprecision(3) << error << std::endl;
        //

        if(state_ == TURN) {
            command.trans_v = 0;
            command.angular_v = ANG_SPEED;
        } else if(state_ == DRIVE) {

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

     void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        odometry_x = odometry->x, odometry_y = odometry->y, odometry_theta = odometry->theta;
        // TODO(EECS467) Implement your handler for new odometry data
        if (state_ == TURN) {
            float dtheta = odometry->theta - theta;
            std::cout << "dtheta: " << dtheta << std::endl;
            if ((dtheta > (M_PI / 2 - 0.08) || dtheta < (-M_PI / 2 + 0.08) ) && fabs(dtheta) < 1.5 * M_PI ) {
                state_ = DRIVE;
                std::cout << "Start driving: " << dtheta << std::endl;
            }
        }
    }

    void handleLidar(const lcm::ReceiveBuffer* buf, const std::string& channel, const lidar_t* lidar) {
        const float min_possible_distance = 0.1;
        const float deviation_range = 0.5; // about 10 degrees
        float min = 100;

        float min_theta_side;
        float min_dist_side;
        for (int i = 0; i < lidar->num_ranges; i++) {
            if (lidar->thetas[i] > M_PI / 2 - deviation_range
            && lidar->thetas[i] < M_PI / 2 + deviation_range
            && lidar->ranges[i] < min && lidar->ranges[i] > min_possible_distance){
                min = lidar->ranges[i];
                min_theta_side = lidar->thetas[i];
                min_dist_side = lidar->ranges[i];
            }
        }
        // this sets dist to keep bot at
        if(initial_lidar_dist < 0){
            std::cout << "Set initial lidar dist to " << initial_lidar_dist << std::endl;
            initial_lidar_dist = min_dist_side;
        }
        min = 100;
        float min_theta_front;
        float min_dist_front;
        for (int i = 0; i < lidar->num_ranges; i++) {
            if (lidar->thetas[i] > -deviation_range
            && lidar->thetas[i] < deviation_range
            && lidar->ranges[i] < min && lidar->ranges[i] > min_possible_distance){
                min = lidar->ranges[i];
                min_theta_front = lidar->thetas[i];
                min_dist_front = lidar->ranges[i];
            }
        }
        // std::cout << "The minimum side distance is: " << min_dist_side << " the theta is: "
        // << min_theta_side <<std::endl;
        // std::cout << "The minimum front distance is: " << min_dist_front << " the theta is: "
        // << min_theta_front <<std::endl;

        if (min_dist_front < 0.1 || min_dist_side < 0.1) state_ = STOP;

        theta_straight = min_theta_side;
        distance_from_wall = min_dist_side;
        
        if (state_ == DRIVE) {
            if (fabs(min_dist_side - min_dist_front) < 0.1) {
                state_ = TURN;
                theta = odometry_theta;
                front_dist_before_turn = min_dist_front;
                current_state++;
                std::cout << "Drive -> Turn transistion" << std::endl;
                // todo: send a turn_t to botgui
                turn_t turn;
                turn.x = odometry_x;
                turn.y = odometry_y;
                std::cout << "Red square at: " << turn.x << ", " << turn.y << std::endl;
                lcmInstance->publish("TURN CHANNEL", &turn);
            }
        } else if (state_ == TURN) {
            // if(lidar->ranges[(M_PI * 100) / 2] > front_dist_before_turn * 1.15){
            //     passed_max = true;
            //     std::cout << "Passed max: front dist: " << front_dist_before_turn << std::endl;
            // }
            // if (lidar->ranges[(M_PI * 100) / 2] - front_dist_before_turn < 0 && passed_max) {
            //     passed_max = false;
            //     state_ = DRIVE;
            //     std::cout << "Turn -> Drive transition" << std::endl;
            // }
            // TODO: TONY commented it out to use odometry for turning for the challenge
        }
    }

private:

    enum State
    {
        TURN,
        DRIVE,
        STOP
    };

    float odometry_x, odometry_y, odometry_theta;

    std::vector<pose_xyt_t> targets_;
    // TODO(EECS467) Initialize the state.
    State state_ = DRIVE;
    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    float theta_straight;
    float front_dist_before_turn;
    float distance_from_wall;
    float passed_max = false;
    float initial_lidar_dist = -1;
    float theta;

    // float odometry_theta;
    int64_t time_offset;
    bool timesync_initialized_;

    // float error_theta_prev; // for checking if the fxxking clamp function is used by odometry

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
    lcmInstance.subscribe(LIDAR_CHANNEL, &MotionController::handleLidar, &controller);
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
        // if(controller.timesync_initialized()){
            mbot_motor_command_t cmd = controller.updateCommand();
            // cmd.left_motor_speed = -TRANS_SPEED; // TONY: FOR TASK 3
            // cmd.right_motor_speed = TRANS_SPEED;
            // std::cout << "Updated motor command\n";
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        // }
    }

    //controller.sp_data.close();

    return 0;
}
