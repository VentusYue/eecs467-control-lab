/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>
#include "stdio.h"

#define PI 3.14159265358979323846

#define DTHETA_THRESH 0.001f

/*******************************************************************************
* mb_initialize_odometry()
*
* TODO: initialize odometry
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
    printf("Initialized robot: x: %f y: %f theta: %f \n\n", x, y, theta);
}


/*******************************************************************************
* mb_update_odometry()
*
* TODO: calculate odometry from internal variables
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){

    // TODO(EECS467): calculate ds and dtheta_encoder properly.
    float ds = (mb_state->left_encoder + mb_state->right_encoder) / 2
                / ENCODER_RES / GEAR_RATIO * PI * WHEEL_DIAMETER;
    float dtheta_encoder = (mb_state->right_encoder - mb_state->left_encoder) / WHEEL_BASE
                            / ENCODER_RES / GEAR_RATIO * PI * WHEEL_DIAMETER;

    // We calculate dtheta based on the sensor readings from
    // the IMU and the encoders, to take the advantage of both sensors.
    // This block of code has been implemented for you.
    float dtheta = 0.0;
    float dtheta_imu = mb_angle_diff_radians(mb_state->last_yaw, mb_state->tb_angles[2]);
    float dtheta_diff = dtheta_imu - dtheta_encoder;
    if(fabs(dtheta_diff) > DTHETA_THRESH){
        dtheta = dtheta_imu;
    }
    else {
        dtheta = dtheta_encoder;
    }
    // dtheta = dtheta_encoder;

    // update odometry
    // TODO(EECS467): calculate the odometry properly, using ds and dtheta
    // TODO: figure out if need to use new_theta or old theta here?
    // printf("NEW THETA %f\n", dtheta + mb_odometry->theta);
    float new_theta = mb_clamp_radians(mb_odometry->theta + dtheta);
    // printf("NEW THETA clamped %f\n", new_theta);
    mb_odometry->x += ds * cos(new_theta);
    mb_odometry->y += ds * sin(new_theta);
    mb_odometry->theta = new_theta;

    // In addition, we calculate the velocities states here.
    // TODO(EECS467): calculate velocities properly

    float delta_t = (float)(now - mb_state->last_time) / 1000000;
    mb_state->turn_velocity = dtheta / delta_t;
    mb_state->fwd_velocity = ds / delta_t;

    mb_state->left_velocity = mb_state->left_encoder / ENCODER_RES / GEAR_RATIO * PI * WHEEL_DIAMETER / delta_t;
    mb_state->right_velocity = mb_state->right_encoder / ENCODER_RES / GEAR_RATIO * PI * WHEEL_DIAMETER / delta_t;
}

/*******************************************************************************
* mb_clamp_radians()
*******************************************************************************/
float mb_clamp_radians(float angle){

    // TODO(EECS467) Clamp the radians to range
    // from -pi to pi
    while (angle < -PI) angle += 2 * PI;
    while (angle > PI) angle -= 2 * PI;
    return angle;
}

float mb_angle_diff_radians(float angle1, float angle2){
    float diff = 0.0;
    // TODO(EECS467) Implement the clamped angle difference.
    diff = angle2 - angle1;
    if (diff < -PI) diff += 2 * PI;
    if (diff > PI) diff -= 2 * PI;
    return diff;
}
