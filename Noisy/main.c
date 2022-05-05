/* Written by Quentin Rossier, 04/04/22 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>

// ...
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Static variable for right and left IR sensors
static int proximity_values_right;
static int proximity_values_left;
// Define current states // No sound => Run Mode / Sound => Still Mode
static int robot_status = 0;
int *robot_status_address = &robot_status;

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        // 1MHz timer clock in order to measure uS.
        NULL,           // Timer callback.
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

// Threads //
// Sensors
static THD_WORKING_AREA(waThdSensor, 128);
static THD_FUNCTION(ThdSensor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	// Update IR sensors
    	proximity_values_right = get_prox(0) + get_prox(1);
    	proximity_values_left = get_prox(6) + get_prox(7);

        chThdYield();
    }
}

// Front LED
static THD_WORKING_AREA(waThdFrontLed, 128);
static THD_FUNCTION(ThdFrontLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	// Front LED (red) turn on if in panic mode
    	if (robot_status == 1)
    	{
    		palSetPad(GPIOD, GPIOD_LED_FRONT);
    	}
        else
        {
        	palClearPad(GPIOD, GPIOD_LED_FRONT);
        }

        chThdSleepMilliseconds(100);
    }
}

// Body LED
static THD_WORKING_AREA(waThdBodyLed, 128);
static THD_FUNCTION(ThdBodyLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	// Body LED (green) turn on if in exploration mode
    	if (robot_status == 0)
    	{
    		palSetPad(GPIOB, GPIOB_LED_BODY);
    	}
        else
        {
        	palClearPad(GPIOB, GPIOB_LED_BODY);
        }
        //palClearPad(GPIOB, GPIOB_LED_BODY);
		// chprintf((BaseSequentialStream*)&SDU1, "%proximity_msg_t %d \r\n", proximity_values, sizeof(proximity_values));
		// palClearPad(GPIOD, GPIOD_LED_FRONT);
		// palTogglePad(GPIOB, GPIOB_LED_BODY);
		// palTogglePad(GPIOD, GPIOD_LED_FRONT);

        chThdSleepMilliseconds(100);
    }
}

// Motors
static THD_WORKING_AREA(waThdMove, 128);
static THD_FUNCTION(ThdMove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	// Check if in exploration mode
    	if (robot_status == 0)
    	{
    		// Check if there is an obstacle
			if (proximity_values_right + proximity_values_left > 100)
			{
				left_motor_set_speed(ROBOT_SPEED_MEDIUM);
				right_motor_set_speed(ROBOT_SPEED_MEDIUM);
			}
			else
			{
				// Obstacle from the left?
				if (proximity_values_right > proximity_values_left)
				{
					// Turn right
					left_motor_set_speed(-ROBOT_SPEED_MEDIUM);
					right_motor_set_speed(ROBOT_SPEED_MEDIUM);
				}
				// Obstacle from the right?
				else
				{
					// Turn left
					left_motor_set_speed(ROBOT_SPEED_MEDIUM);
					right_motor_set_speed(-ROBOT_SPEED_MEDIUM);
				}
			}

    	}
    	// Panic mode
    	else
    	{
    		left_motor_set_speed(ROBOT_SPEED_NULL);
    		right_motor_set_speed(ROBOT_SPEED_NULL);
    	}
    	chThdSleepMilliseconds(10);
    }
}

// Speaker
static THD_WORKING_AREA(waThdSpeaker, 128);
static THD_FUNCTION(ThdSpeaker, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        // Make noise if in panic mode
    	if (robot_status == 1)
    	{
    		dac_play(440);
    	    //playMelodyStart();
    	    //playMelody(MARIO, ML_SIMPLE_PLAY, NULL);
    	    //waitMelodyHasFinished();
    	    //playNote(NOTE_A4, 50);
    	    //waitMelodyHasFinished();
    	}
        else
        {
        	//dac_stopI();
        }
        chThdSleepMilliseconds(100);
    }
}

int main(void)
{
	// Inits
    halInit();
    chSysInit();
    mpu_init();
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    // Motors
    motors_init();
    // IR sensors
    proximity_start();
    // Speaker
    dac_start();
    // Inits the Inter Process Communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Create Threads //
    // Microphones //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
    // Sensors // Set to highest priority
    chThdCreateStatic(waThdSensor, sizeof(waThdSensor), NORMALPRIO , ThdSensor, NULL);
    chThdSetPriority(NORMALPRIO+1);
    // LEDs
    chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO , ThdFrontLed, NULL);
    chThdCreateStatic(waThdBodyLed, sizeof(waThdBodyLed), NORMALPRIO, ThdBodyLed, NULL);
    // Motors
    chThdCreateStatic(waThdMove, sizeof(waThdMove), NORMALPRIO, ThdMove, NULL);
	// Speaker
    chThdCreateStatic(waThdSpeaker, sizeof(waThdSpeaker), NORMALPRIO, ThdSpeaker, NULL);

    // Wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);

    /* Infinite loop. */
    while (1) {
    	// ...
    	wait_send_to_computer();
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
