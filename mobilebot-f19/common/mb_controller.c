#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    mb_load_controller_config();
    mb_initialize_left();
    mb_initialize_right();
    return 0;
}

void mb_initialize_left(){
    left_pid = rc_filter_empty();
    rc_filter_pid(
        &left_pid,
        left_pid_params.kp,
        left_pid_params.ki,
        left_pid_params.kd,
        1.0 / left_pid_params.dFilterHz,
        1.0 / SAMPLE_RATE_HZ);
    rc_filter_enable_saturation(&left_pid, -1.0, 1.0);
}

void mb_initialize_right(){
    right_pid = rc_filter_empty();
    rc_filter_pid(
        &right_pid,
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        1.0/right_pid_params.dFilterHz,
        1.0/SAMPLE_RATE_HZ);
    rc_filter_enable_saturation(&right_pid,-1.0, 1.0);
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your  PID controllers
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){

    //set points for the fwd and turn velocities
    float fwd_sp, turn_sp, left_sp, right_sp;
    
    //printf("fwd SP%f: \n", mb_setpoints->fwd_velocity);
    fwd_sp = mb_setpoints->fwd_velocity;
    turn_sp = mb_setpoints->turn_velocity;

    // TODO(EECS467) calculate left wheel and right wheel set point velocities
    // properly.
    //printf("LEFT SP%f: \n", left_sp);
    left_sp = fwd_sp - WHEEL_BASE * turn_sp / 2;
    right_sp = fwd_sp + WHEEL_BASE * turn_sp / 2;
    
    // mb_setpoints->left_sp = left_sp; // commented out for task 4
    // mb_setpoints->right_sp = right_sp;
    // left_sp = mb_setpoints->left_sp; // TODO :task 4 only
    // right_sp = mb_setpoints->right_sp;

    // PID controllers for left and right wheel.
    //printf("LSP: %f, LVEL: %f\n", left_sp, mb_state->left_velocity);
    mb_state->left_cmd = rc_filter_march(&left_pid, left_sp - mb_state->left_velocity);
    mb_state->right_cmd = rc_filter_march(&right_pid, right_sp - mb_state->right_velocity);
    
    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    rc_filter_free(&right_pid);
    rc_filter_free(&left_pid);
    return 0;
}