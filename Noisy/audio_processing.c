#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>


#define MIN_VALUE_THRESHOLD 180

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){
	/*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*/
	// Noise Threshold, the robot change status
	if ((data[MIC_FRONT] > MIN_VALUE_THRESHOLD) || (getCounterState()))
	{
		changeRobotStatusToPanic();
	}
	else
	{
		changeRobotStatusToExploration();
	}

	static uint16_t nb_samples = 0;
}

