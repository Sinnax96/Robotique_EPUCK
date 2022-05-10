#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

/* Define global constants */
#define ROBOT_SPEED_MAX		1100
#define ROBOT_SPEED_PANIC	700
#define ROBOT_SPEED_MEDIUM	300
#define ROBOT_SPEED_SLOW	100
#define ROBOT_SPEED_NULL	0
#define ROBOT_STATUS_EXPLORATION	0
#define ROBOT_STATUS_PANIC	1

int getCounterState(void);
int getRobotStatus(void);
int changeRobotStatusToExploration(void);
int changeRobotStatusToPanic(void);

#endif
