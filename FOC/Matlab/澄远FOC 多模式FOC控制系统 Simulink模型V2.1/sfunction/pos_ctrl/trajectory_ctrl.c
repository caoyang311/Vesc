#include <string.h>
#include "trajectory_ctrl.h"

void PosCtrlInit(PosCtrlHandle_t *pout)
{
	pout->start_angle = 0.0f;
	pout->angle_step = 0.0f;
	pout->final_angle = 0.0f;
	pout->move_duration = 0.0f;
	pout->sub_step[0] = 0.0f;
	pout->sub_step[1] = 0.0f;
	pout->sub_step[2] = 0.0f;
	pout->sub_step[3] = 0.0f;
	pout->sub_step[4] = 0.0f;
	pout->sub_step[5] = 0.0f;
	pout->sub_step[6] = 0.0f;

	pout->sub_step_duration = 0.0f;
	pout->elapse_time = 0.0f;
	pout->sample_time = 1.0f / POS_CTRL_FRQ;
	pout->jerk = 0.0f;
	pout->cruise_spd = 0.0f;
	pout->acceleration = 0.0f;
	pout->omega = 0.0f;
	pout->theta = 0.0f;
	pout->tc_sts = TC_STATE_IDLE;
	
}

uint8 PosCtrPlan(float start_pos, float tar_pos, float duration, PosCtrlHandle_t *pin, PosCtrlHandle_t *pout)
{
	float a;
	float min_duration;

	if((pin->tc_sts != TC_STATE_IDLE) || (duration <= 0))
		return FALSE;

	/* set position and duration info */
	pin->start_angle = start_pos;
	pin->angle_step = tar_pos - start_pos;
	pin->final_angle = tar_pos;
	min_duration = 9.0f * pin->sample_time;
	pin->move_duration = (float)((int32)(duration / min_duration))*min_duration;

	/* set end time for every sub stage */
	pin->sub_step_duration = pin->move_duration / 9.0f;
	pin->sub_step[0] = 1 * pin->sub_step_duration;
	pin->sub_step[1] = 2 * pin->sub_step_duration;
	pin->sub_step[2] = 3 * pin->sub_step_duration;
	pin->sub_step[3] = 6 * pin->sub_step_duration;
	pin->sub_step[4] = 7 * pin->sub_step_duration;
	pin->sub_step[5] = 8 * pin->sub_step_duration;
	pin->sub_step[6] = pin->move_duration;

	/* caculate jerk and cruise speed */
	a = pin->sub_step_duration;
	pin->jerk = pin->angle_step / (12 * a * a * a);
	pin->cruise_spd = 2*pin->jerk * a * a;

	/* set run time and posctrl state */
	pin->elapse_time = 0.0f;
	pin->tc_sts = TC_STATE_GOING;

	/* set output info */
	pin->acceleration = 0.0f;
	pin->omega = 0.0f;
	pin->theta = pin->start_angle;

	memcpy(pout, pin, sizeof(PosCtrlHandle_t));

	return TRUE;
}

uint8 PosCtrPlanExec(PosCtrlHandle_t *pin, PosCtrlHandle_t *pout)
{
	float jerk;

	/* only run in GOING state  */
	if(pin->tc_sts != TC_STATE_GOING)
		return FALSE;

	jerk = 0.0f;

	/* set jerk for corresponding sub stage */
	if(pin->elapse_time < pin->sub_step[0])
		jerk = pin->jerk;
	else if(pin->elapse_time < pin->sub_step[1])
		jerk = 0.0f;
	else if(pin->elapse_time < pin->sub_step[2])
		jerk = -pin->jerk;
	else if(pin->elapse_time < pin->sub_step[3])
	{
		pin->acceleration = 0.0f;
		pin->omega = pin->cruise_spd;
	}
	else if(pin->elapse_time < pin->sub_step[4])
		jerk = -pin->jerk;
	else if(pin->elapse_time < pin->sub_step[5])
		jerk = 0.0f;
	else if(pin->elapse_time < pin->sub_step[6])
		jerk = pin->jerk;
	else
	{
		if(pin->angle_step >= 0)
		{
			if(pin->theta > pin->final_angle)
				pin->theta = pin->final_angle;
		}
		else
		{
			if(pin->theta < pin->final_angle)
				pin->theta = pin->final_angle;
		}

		pin->tc_sts = TC_STATE_REACHED;
	}

	if(pin->tc_sts == TC_STATE_GOING)
	{
		/* The integral of jerk is equal to acceleration */
		pin->acceleration += jerk * pin->sample_time;
		/* The integral of acceleration is equal to velocity */
		pin->omega += pin->acceleration * pin->sample_time;
		/* The integral of velocity is equal to displacement */
		pin->theta += pin->omega * pin->sample_time;

		if(pin->angle_step >= 0)
		{
			if(pin->theta > pin->final_angle)
				pin->theta = pin->final_angle;
		}
		else
		{
			if(pin->theta < pin->final_angle)
				pin->theta = pin->final_angle;
		}
	}

	/* update run time */
	pin->elapse_time += pin->sample_time;
	if(pin->elapse_time > pin->move_duration)
//	if(pin->elapse_time > pin->move_duration + pin->sample_time)
	{
		pin->tc_sts = TC_STATE_IDLE;
		pin->elapse_time = 0.0f;
	}
	
	memcpy(pout, pin, sizeof(PosCtrlHandle_t));

	return TRUE;
}

