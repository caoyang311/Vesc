#ifndef TRAJECTORY_H_
#define	TRAJECTORY_H_

#include "data_type.h"
#define	POS_CTRL_FRQ		500

#define	TC_STATE_IDLE		0
#define	TC_STATE_GOING		1
#define	TC_STATE_REACHED	2	// reach target position

typedef struct
{
	float	start_angle;		// start angle, unit: rad
	float	angle_step;			// rotate angle, uint: rad
	float	final_angle;		// final angle, uint: rad
	float	move_duration;		// total duration time
	float	sub_step[7];		// 7 sub stage
	float	sub_step_duration;	// every sub stage duration time
	float	elapse_time;		// run time
	float	sample_time;		// every times run time
	float	jerk;				// jerk
	float	cruise_spd;			// cruise stage speed
	float	acceleration;		// acceleration
	float	omega;				// speed
	float	theta;				// theta
	
	uint8	tc_sts;				// trajectory controler state
}PosCtrlHandle_t;

void PosCtrlInit(PosCtrlHandle_t *pout);
uint8 PosCtrPlan(float start_pos, float tar_pos, float duration, PosCtrlHandle_t *pin, PosCtrlHandle_t *pout);
uint8 PosCtrPlanExec(PosCtrlHandle_t *pin, PosCtrlHandle_t *pout);

#endif
