/* Written by Quentin Rossier and Quentin Boinay, 11/05/22 */

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
// Define current states // No sound => Exploration Mode / Sound => Panic Mode
static int robot_status = ROBOT_STATUS_EXPLORATION;
// Define EndCounter
static int CounterState = 0;

// Fonctions
int getCounterState(){
  return CounterState;
}

int getRobotStatus(){
  return robot_status;
}

void changeRobotStatusToExploration(){
  robot_status = ROBOT_STATUS_EXPLORATION;
}

void changeRobotStatusToPanic(){
  robot_status = ROBOT_STATUS_PANIC;
}


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
      // Front LED (red) blink if in panic mode
      if (robot_status == ROBOT_STATUS_PANIC)
      {
        palTogglePad(GPIOD, GPIOD_LED_FRONT);
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
      // Body LED (green) turn on if in exploration mode and blink if turning
      if (robot_status == ROBOT_STATUS_EXPLORATION)
      {
        if (proximity_values_right + proximity_values_left > 100)
        {
          palTogglePad(GPIOB, GPIOB_LED_BODY);
        }
        else
        {
          palSetPad(GPIOB, GPIOB_LED_BODY);
        }
      }
      else
      {
        palClearPad(GPIOB, GPIOB_LED_BODY);
      }
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
      if (robot_status == ROBOT_STATUS_EXPLORATION)
      {
        // Check if there is an obstacle
        if (proximity_values_right + proximity_values_left < 100)
        {
          left_motor_set_speed(ROBOT_SPEED_MEDIUM);
          right_motor_set_speed(ROBOT_SPEED_MEDIUM);
        }
        else
        {
          // Obstacle from the right?
          if (proximity_values_right > proximity_values_left)
          {
            // Turn left
            left_motor_set_speed(-ROBOT_SPEED_MEDIUM);
            right_motor_set_speed(ROBOT_SPEED_MEDIUM);
          }
          // Obstacle from the left?
          else
          {
            // Turn right
            left_motor_set_speed(ROBOT_SPEED_MEDIUM);
            right_motor_set_speed(-ROBOT_SPEED_MEDIUM);
          }
        }
      }
      // Panic mode
      else
      {
        left_motor_set_speed(ROBOT_SPEED_MEDIUM);
        right_motor_set_speed(ROBOT_SPEED_MAX);
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
      // Play death sound when in Panic mode
      if (robot_status == ROBOT_STATUS_PANIC)
      {
    	  stopCurrentMelody();
    	  playNote(NOTE_G6,400);
    	  playNote(NOTE_C6,400);
      }
      // Play music while exploration
      else
      {
    	  playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);
      }
      chThdSleepMilliseconds(100);
    }
}

// Counter
static THD_WORKING_AREA(waThdCounter, 128);
static THD_FUNCTION(ThdCounter, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    int Counter = 0;
    while(1){
      if(robot_status == ROBOT_STATUS_PANIC)
      {
        CounterState = 1;
        if(Counter < 4)
        {
          Counter += 1;
          chThdSleepMilliseconds(1000);
        }
        else{
          CounterState = 0;
          Counter = 0;
        }
      }

      chThdSleepMilliseconds(30);
    }
}

int main(void)
{
    // Inits
    halInit();
    chSysInit();
    //mpu_init();
    // Starts the serial communication
    serial_start();
    // Starts the USB communication
   // usb_start();
    // Starts timer 12
    //timer12_start();
    // Motors
    motors_init();
    // IR sensors
    proximity_start();

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
    // Counter
    chThdCreateStatic(waThdCounter, sizeof(waThdCounter), NORMALPRIO , ThdCounter, NULL);

    // IR sensors calibration
    calibrate_ir();
    // Speaker
     dac_start();
     // Melody
     playMelodyStart();

    // Wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);

    /* Infinite loop. */
    while (1){
      chThdSleepMilliseconds(2000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

