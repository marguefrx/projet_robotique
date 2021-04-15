#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

int16_t pi_regulator(float distance, float goal)
{
    float error = 0;
    float speed = 0;

    static float sum_error = 0;

    error = distance - goal;
    sum_error += error;

    if(sum_error > MAX_SUM_ERROR) {
        sum_error = MAX_SUM_ERROR;
    } else if (sum_error < -MAX_SUM_ERROR) {
        sum_error = -MAX_SUM_ERROR;
    }

    speed = KP * error + KI *sum_error;

    return (int16_t) speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0, speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        if (abs(speed_correction) < ROTATION_THRESHOLD) {
            speed_correction = 0;
        }

        right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
        left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        
        //applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
