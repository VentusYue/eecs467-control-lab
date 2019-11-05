#include <lcm/lcm.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "../lcmtypes/mbot_motor_command_t.h"

bool continue_running=true;

typedef struct state
{
  lcm_t *lcm;
  char *channel;
} state_t;

void signal_handler(int v)
{
    continue_running = false;
}

void motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const mbot_motor_command_t *msg, void *user){
	mb_setpoints.fwd_velocity = msg->trans_v;
	mb_setpoints.turn_velocity = msg->angular_v;
}



int main(int argc, char const *argv[]) {
    struct sigaction signal_action = { 0 };
    signal_action.sa_handler = signal_handler;
    sigaction(SIGINT, &signal_action, NULL);

    state_t *st = calloc(1, sizeof(state_t));
    st->lcm = lcm_create(NULL);
    st->channel = "MBOT_MOTOR_COMMAND";

    mbot_motor_command_t mbot_cmd;
    mbot_cmd.trans_v = 0.1;
    mbot_cmd.angular_v = 0;

    mbot_motor_command_t_subscribe(st->lcm,
                                   st->channel,
                                   motor_command_handler,
                                   NULL);
    mbot_motor_command_t_publish(st->lcm, st->channel,&mbot_cmd);
    while (continue_running) {
        lcm_handle(st->lcm);
    }
    lcm_destroy(st->lcm);

    return 0;
}
