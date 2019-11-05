#ifndef TP_H
#define TP_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <rc/adc.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/dsm.h>
#include <rc/encoder.h>
#include <rc/gpio.h>
#include <rc/i2c.h>
#include <rc/led.h>
#include <rc/math.h>
#include <rc/model.h>
#include <rc/motor.h>
#include <rc/mpu.h>
#include <rc/pru.h>
#include <rc/pthread.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/version.h>

#include <lcm/lcm.h>
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/mbot_imu_t.h"
#include "../lcmtypes/mbot_motor_command_t.h"
#include "../lcmtypes/odometry_t.h"
#include "../lcmtypes/oled_message_t.h"
#include "../lcmtypes/timestamp_t.h"
#include "../lcmtypes/reset_odometry_t.h"
#include "../lcmtypes/setpoint_t.h"

#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../common/mb_motor.h"

#endif
