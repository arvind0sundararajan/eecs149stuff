
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../src/sc_types.h"
#include "Robot_template.h"
/*! \file Implementation of the state machine 'robot_template'
*/

/* prototypes of all internal functions */
static sc_boolean check_main_region_OFF_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_OFF_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_ON_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_ON_tr1_tr1(const Robot_template* handle);
static sc_boolean check_main_region_ON_r1_DRIVING_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_ON_r1_DRIVING_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_ON_r1_TURNING_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_ON_r1_TURNING_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_tr1_tr1(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_r1_BACKUP_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_r1_BACKUP_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_r1_BACKUP_tr1_tr1(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_r1_TURNL_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_AVOID_r1_TURNR_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_tr0_tr0(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_tr1_tr1(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_tr2_tr2(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_REORIENT_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_DRIVING_lr0_lr0(const Robot_template* handle);
static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0_tr0(const Robot_template* handle);
static void effect_main_region_OFF_lr0_lr0(Robot_template* handle);
static void effect_main_region_OFF_tr0(Robot_template* handle);
static void effect_main_region_ON_tr0(Robot_template* handle);
static void effect_main_region_ON_tr1(Robot_template* handle);
static void effect_main_region_ON_r1_DRIVING_lr0_lr0(Robot_template* handle);
static void effect_main_region_ON_r1_DRIVING_tr0(Robot_template* handle);
static void effect_main_region_ON_r1_TURNING_lr0_lr0(Robot_template* handle);
static void effect_main_region_ON_r1_TURNING_tr0(Robot_template* handle);
static void effect_main_region_AVOID_tr0(Robot_template* handle);
static void effect_main_region_AVOID_tr1(Robot_template* handle);
static void effect_main_region_AVOID_r1_BACKUP_lr0_lr0(Robot_template* handle);
static void effect_main_region_AVOID_r1_BACKUP_tr0(Robot_template* handle);
static void effect_main_region_AVOID_r1_BACKUP_tr1(Robot_template* handle);
static void effect_main_region_AVOID_r1_TURNL_lr0_lr0(Robot_template* handle);
static void effect_main_region_AVOID_r1_TURNR_lr0_lr0(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_tr0(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_tr1(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_tr2(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_r1_REORIENT_lr0_lr0(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_lr0_lr0(Robot_template* handle);
static void effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0(Robot_template* handle);
static void enseq_main_region_OFF_default(Robot_template* handle);
static void enseq_main_region_ON_default(Robot_template* handle);
static void enseq_main_region_ON_r1_DRIVING_default(Robot_template* handle);
static void enseq_main_region_ON_r1_TURNING_default(Robot_template* handle);
static void enseq_main_region_AVOID_default(Robot_template* handle);
static void enseq_main_region_AVOID_r1_BACKUP_default(Robot_template* handle);
static void enseq_main_region_AVOID_r1_TURNL_default(Robot_template* handle);
static void enseq_main_region_AVOID_r1_TURNR_default(Robot_template* handle);
static void enseq_main_region_REORIENT_SEQUENCE_default(Robot_template* handle);
static void enseq_main_region_REORIENT_SEQUENCE_r1_REORIENT_default(Robot_template* handle);
static void enseq_main_region_REORIENT_SEQUENCE_r1_DRIVING_default(Robot_template* handle);
static void enseq_main_region_default(Robot_template* handle);
static void enseq_main_region_ON_r1_default(Robot_template* handle);
static void enseq_main_region_AVOID_r1_default(Robot_template* handle);
static void enseq_main_region_REORIENT_SEQUENCE_r1_default(Robot_template* handle);
static void exseq_main_region_OFF(Robot_template* handle);
static void exseq_main_region_ON(Robot_template* handle);
static void exseq_main_region_ON_r1_DRIVING(Robot_template* handle);
static void exseq_main_region_ON_r1_TURNING(Robot_template* handle);
static void exseq_main_region_AVOID(Robot_template* handle);
static void exseq_main_region_AVOID_r1_BACKUP(Robot_template* handle);
static void exseq_main_region_AVOID_r1_TURNL(Robot_template* handle);
static void exseq_main_region_AVOID_r1_TURNR(Robot_template* handle);
static void exseq_main_region_REORIENT_SEQUENCE(Robot_template* handle);
static void exseq_main_region_REORIENT_SEQUENCE_r1_REORIENT(Robot_template* handle);
static void exseq_main_region_REORIENT_SEQUENCE_r1_DRIVING(Robot_template* handle);
static void exseq_main_region(Robot_template* handle);
static void exseq_main_region_ON_r1(Robot_template* handle);
static void exseq_main_region_AVOID_r1(Robot_template* handle);
static void exseq_main_region_REORIENT_SEQUENCE_r1(Robot_template* handle);
static void react_main_region_OFF(Robot_template* handle);
static void react_main_region_ON_r1_DRIVING(Robot_template* handle);
static void react_main_region_ON_r1_TURNING(Robot_template* handle);
static void react_main_region_AVOID_r1_BACKUP(Robot_template* handle);
static void react_main_region_AVOID_r1_TURNL(Robot_template* handle);
static void react_main_region_AVOID_r1_TURNR(Robot_template* handle);
static void react_main_region_REORIENT_SEQUENCE_r1_REORIENT(Robot_template* handle);
static void react_main_region_REORIENT_SEQUENCE_r1_DRIVING(Robot_template* handle);
static void react_main_region__entry_Default(Robot_template* handle);
static void react_main_region_ON_r1__entry_Default(Robot_template* handle);
static void react_main_region_AVOID_r1__entry_Default(Robot_template* handle);
static void react_main_region_REORIENT_SEQUENCE_r1__entry_Default(Robot_template* handle);
static void clearInEvents(Robot_template* handle);
static void clearOutEvents(Robot_template* handle);


void robot_template_init(Robot_template* handle)
{
		sc_integer i;
	
		for (i = 0; i < ROBOT_TEMPLATE_MAX_ORTHOGONAL_STATES; ++i)
		{
			handle->stateConfVector[i] = Robot_template_last_state;
		}
		
		
		handle->stateConfVectorPosition = 0;
	
		clearInEvents(handle);
		clearOutEvents(handle);
	
		/* Default init sequence for statechart robot_template */
		handle->iface.distance = 0.0;
		handle->iface.prev_encoder = read_encoder();
		handle->iface.angle = 0.0;
		handle->iface.cumulative_angle = 0.0;
		handle->iface.turnL = bool_false;
		handle->iface.turnR = bool_false;
		handle->iface.reorienting = bool_false;
}

void robot_template_enter(Robot_template* handle)
{
	/* Default enter sequence for statechart robot_template */
	enseq_main_region_default(handle);
}

void robot_template_exit(Robot_template* handle)
{
	/* Default exit sequence for statechart robot_template */
	exseq_main_region(handle);
}

sc_boolean robot_template_isActive(const Robot_template* handle)
{
	sc_boolean result = bool_false;
	int i;
	
	for(i = 0; i < ROBOT_TEMPLATE_MAX_ORTHOGONAL_STATES; i++)
	{
		result = result || handle->stateConfVector[i] != Robot_template_last_state;
	}
	
	return result;
}

/* 
 * Always returns 'false' since this state machine can never become final.
 */
sc_boolean robot_template_isFinal(const Robot_template* handle)
{
   return bool_false;
}

static void clearInEvents(Robot_template* handle)
{
}

static void clearOutEvents(Robot_template* handle)
{
}

void robot_template_runCycle(Robot_template* handle)
{
	
	clearOutEvents(handle);
	for (handle->stateConfVectorPosition = 0;
		handle->stateConfVectorPosition < ROBOT_TEMPLATE_MAX_ORTHOGONAL_STATES;
		handle->stateConfVectorPosition++)
		{
			
		switch (handle->stateConfVector[handle->stateConfVectorPosition])
		{
		case Robot_template_main_region_OFF:
		{
			react_main_region_OFF(handle);
			break;
		}
		case Robot_template_main_region_ON_r1_DRIVING:
		{
			react_main_region_ON_r1_DRIVING(handle);
			break;
		}
		case Robot_template_main_region_ON_r1_TURNING:
		{
			react_main_region_ON_r1_TURNING(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_BACKUP:
		{
			react_main_region_AVOID_r1_BACKUP(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNL:
		{
			react_main_region_AVOID_r1_TURNL(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNR:
		{
			react_main_region_AVOID_r1_TURNR(handle);
			break;
		}
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT:
		{
			react_main_region_REORIENT_SEQUENCE_r1_REORIENT(handle);
			break;
		}
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING:
		{
			react_main_region_REORIENT_SEQUENCE_r1_DRIVING(handle);
			break;
		}
		default:
			break;
		}
	}
	
	clearInEvents(handle);
}


sc_boolean robot_template_isStateActive(const Robot_template* handle, Robot_templateStates state)
{
	sc_boolean result = bool_false;
	switch (state)
	{
		case Robot_template_main_region_OFF :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_OFF] == Robot_template_main_region_OFF
			);
			break;
		case Robot_template_main_region_ON :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON] >= Robot_template_main_region_ON
				&& handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON] <= Robot_template_main_region_ON_r1_TURNING);
			break;
		case Robot_template_main_region_ON_r1_DRIVING :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON_R1_DRIVING] == Robot_template_main_region_ON_r1_DRIVING
			);
			break;
		case Robot_template_main_region_ON_r1_TURNING :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON_R1_TURNING] == Robot_template_main_region_ON_r1_TURNING
			);
			break;
		case Robot_template_main_region_AVOID :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID] >= Robot_template_main_region_AVOID
				&& handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID] <= Robot_template_main_region_AVOID_r1_TURNR);
			break;
		case Robot_template_main_region_AVOID_r1_BACKUP :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_BACKUP] == Robot_template_main_region_AVOID_r1_BACKUP
			);
			break;
		case Robot_template_main_region_AVOID_r1_TURNL :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_TURNL] == Robot_template_main_region_AVOID_r1_TURNL
			);
			break;
		case Robot_template_main_region_AVOID_r1_TURNR :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_TURNR] == Robot_template_main_region_AVOID_r1_TURNR
			);
			break;
		case Robot_template_main_region_REORIENT_SEQUENCE :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE] >= Robot_template_main_region_REORIENT_SEQUENCE
				&& handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE] <= Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING);
			break;
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE_R1_REORIENT] == Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT
			);
			break;
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING :
			result = (sc_boolean) (handle->stateConfVector[SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE_R1_DRIVING] == Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING
			);
			break;
		default:
			result = bool_false;
			break;
	}
	return result;
}



float robot_templateIface_get_distance(const Robot_template* handle)
{
	return handle->iface.distance;
}
void robot_templateIface_set_distance(Robot_template* handle, float value)
{
	handle->iface.distance = value;
}
uint16_t robot_templateIface_get_prev_encoder(const Robot_template* handle)
{
	return handle->iface.prev_encoder;
}
void robot_templateIface_set_prev_encoder(Robot_template* handle, uint16_t value)
{
	handle->iface.prev_encoder = value;
}
float robot_templateIface_get_angle(const Robot_template* handle)
{
	return handle->iface.angle;
}
void robot_templateIface_set_angle(Robot_template* handle, float value)
{
	handle->iface.angle = value;
}
float robot_templateIface_get_cumulative_angle(const Robot_template* handle)
{
	return handle->iface.cumulative_angle;
}
void robot_templateIface_set_cumulative_angle(Robot_template* handle, float value)
{
	handle->iface.cumulative_angle = value;
}
sc_boolean robot_templateIface_get_turnL(const Robot_template* handle)
{
	return handle->iface.turnL;
}
void robot_templateIface_set_turnL(Robot_template* handle, sc_boolean value)
{
	handle->iface.turnL = value;
}
sc_boolean robot_templateIface_get_turnR(const Robot_template* handle)
{
	return handle->iface.turnR;
}
void robot_templateIface_set_turnR(Robot_template* handle, sc_boolean value)
{
	handle->iface.turnR = value;
}
sc_boolean robot_templateIface_get_reorienting(const Robot_template* handle)
{
	return handle->iface.reorienting;
}
void robot_templateIface_set_reorienting(Robot_template* handle, sc_boolean value)
{
	handle->iface.reorienting = value;
}

/* implementations of all internal functions */

static sc_boolean check_main_region_OFF_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_OFF_tr0_tr0(const Robot_template* handle)
{
	return is_button_press();
}

static sc_boolean check_main_region_ON_tr0_tr0(const Robot_template* handle)
{
	return is_button_press();
}

static sc_boolean check_main_region_ON_tr1_tr1(const Robot_template* handle)
{
	return (is_right_bumper() || is_left_bumper() || is_front_bumper()) ? bool_true : bool_false;
}

static sc_boolean check_main_region_ON_r1_DRIVING_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_ON_r1_DRIVING_tr0_tr0(const Robot_template* handle)
{
	return (handle->iface.distance >= 0.5) ? bool_true : bool_false;
}

static sc_boolean check_main_region_ON_r1_TURNING_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_ON_r1_TURNING_tr0_tr0(const Robot_template* handle)
{
	return (handle->iface.angle >= 90) ? bool_true : bool_false;
}

static sc_boolean check_main_region_AVOID_tr0_tr0(const Robot_template* handle)
{
	return is_button_press();
}

static sc_boolean check_main_region_AVOID_tr1_tr1(const Robot_template* handle)
{
	return (handle->iface.angle >= 45 || handle->iface.angle <= -45) ? bool_true : bool_false;
}

static sc_boolean check_main_region_AVOID_r1_BACKUP_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_AVOID_r1_BACKUP_tr0_tr0(const Robot_template* handle)
{
	return (handle->iface.distance >= 0.1 && handle->iface.turnL) ? bool_true : bool_false;
}

static sc_boolean check_main_region_AVOID_r1_BACKUP_tr1_tr1(const Robot_template* handle)
{
	return (handle->iface.distance >= 0.1 && handle->iface.turnR) ? bool_true : bool_false;
}

static sc_boolean check_main_region_AVOID_r1_TURNL_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_AVOID_r1_TURNR_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_tr0_tr0(const Robot_template* handle)
{
	return (fmod((handle->iface.angle + handle->iface.cumulative_angle),360) <= 2 && fmod((handle->iface.angle + handle->iface.cumulative_angle),360) >= -2 && handle->iface.reorienting) ? bool_true : bool_false;
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_tr1_tr1(const Robot_template* handle)
{
	return (is_right_bumper() || is_left_bumper() || is_front_bumper()) ? bool_true : bool_false;
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_tr2_tr2(const Robot_template* handle)
{
	return is_button_press();
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_REORIENT_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_DRIVING_lr0_lr0(const Robot_template* handle)
{
	return bool_true;
}

static sc_boolean check_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0_tr0(const Robot_template* handle)
{
	return (handle->iface.distance >= 0.25) ? bool_true : bool_false;
}

static void effect_main_region_OFF_lr0_lr0(Robot_template* handle)
{
	stop_kobuki();
	print_state(OFF);
}

static void effect_main_region_OFF_tr0(Robot_template* handle)
{
	exseq_main_region_OFF(handle);
	handle->iface.prev_encoder = read_encoder();
	enseq_main_region_ON_default(handle);
}

static void effect_main_region_ON_tr0(Robot_template* handle)
{
	exseq_main_region_ON(handle);
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	enseq_main_region_OFF_default(handle);
}

static void effect_main_region_ON_tr1(Robot_template* handle)
{
	exseq_main_region_ON(handle);
	handle->iface.turnL = is_left_bumper();
	handle->iface.turnR = !handle->iface.turnL;
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	handle->iface.prev_encoder = read_encoder();
	enseq_main_region_AVOID_default(handle);
}

static void effect_main_region_ON_r1_DRIVING_lr0_lr0(Robot_template* handle)
{
	drive_kobuki(bool_true);
	print_state(DRIVING);
	handle->iface.distance = update_dist(handle->iface.distance, handle->iface.prev_encoder, bool_true);
	handle->iface.prev_encoder = read_encoder();
	print_dist(handle->iface.distance);
}

static void effect_main_region_ON_r1_DRIVING_tr0(Robot_template* handle)
{
	exseq_main_region_ON_r1_DRIVING(handle);
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	start_gyro();
	enseq_main_region_ON_r1_TURNING_default(handle);
}

static void effect_main_region_ON_r1_TURNING_lr0_lr0(Robot_template* handle)
{
	turn_kobuki(bool_false);
	print_state(LEFT);
	handle->iface.angle = read_gyro();
	print_angle(handle->iface.angle);
	print_turn(LEFT_TURN);
}

static void effect_main_region_ON_r1_TURNING_tr0(Robot_template* handle)
{
	exseq_main_region_ON_r1_TURNING(handle);
	stop_gyro();
	stop_kobuki();
	handle->iface.angle = 0;
	enseq_main_region_ON_r1_DRIVING_default(handle);
}

static void effect_main_region_AVOID_tr0(Robot_template* handle)
{
	exseq_main_region_AVOID(handle);
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	stop_gyro();
	enseq_main_region_OFF_default(handle);
}

static void effect_main_region_AVOID_tr1(Robot_template* handle)
{
	exseq_main_region_AVOID(handle);
	stop_gyro();
	handle->iface.distance = 0;
	stop_kobuki();
	handle->iface.cumulative_angle += handle->iface.angle;
	handle->iface.angle = 0;
	handle->iface.prev_encoder = read_encoder();
	enseq_main_region_REORIENT_SEQUENCE_default(handle);
}

static void effect_main_region_AVOID_r1_BACKUP_lr0_lr0(Robot_template* handle)
{
	drive_kobuki(bool_false);
	print_state(BACKUP);
	handle->iface.distance = update_dist(handle->iface.distance, handle->iface.prev_encoder, bool_false);
	handle->iface.prev_encoder = read_encoder();
	print_dist(handle->iface.distance);
}

static void effect_main_region_AVOID_r1_BACKUP_tr0(Robot_template* handle)
{
	exseq_main_region_AVOID_r1_BACKUP(handle);
	start_gyro();
	enseq_main_region_AVOID_r1_TURNL_default(handle);
}

static void effect_main_region_AVOID_r1_BACKUP_tr1(Robot_template* handle)
{
	exseq_main_region_AVOID_r1_BACKUP(handle);
	start_gyro();
	enseq_main_region_AVOID_r1_TURNR_default(handle);
}

static void effect_main_region_AVOID_r1_TURNL_lr0_lr0(Robot_template* handle)
{
	turn_kobuki(bool_true);
	handle->iface.angle = read_gyro();
	print_angle(handle->iface.angle);
	print_turn(RIGHT_TURN);
}

static void effect_main_region_AVOID_r1_TURNR_lr0_lr0(Robot_template* handle)
{
	turn_kobuki(bool_false);
	handle->iface.angle = read_gyro();
	print_angle(handle->iface.angle);
	print_turn(LEFT_TURN);
}

static void effect_main_region_REORIENT_SEQUENCE_tr0(Robot_template* handle)
{
	exseq_main_region_REORIENT_SEQUENCE(handle);
	stop_gyro();
	handle->iface.cumulative_angle = 0;
	handle->iface.angle = 0;
	handle->iface.reorienting = bool_false;
	enseq_main_region_ON_default(handle);
}

static void effect_main_region_REORIENT_SEQUENCE_tr1(Robot_template* handle)
{
	exseq_main_region_REORIENT_SEQUENCE(handle);
	handle->iface.turnL = is_left_bumper();
	handle->iface.turnR = !handle->iface.turnL;
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	handle->iface.prev_encoder = read_encoder();
	enseq_main_region_AVOID_default(handle);
}

static void effect_main_region_REORIENT_SEQUENCE_tr2(Robot_template* handle)
{
	exseq_main_region_REORIENT_SEQUENCE(handle);
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	stop_gyro();
	handle->iface.reorienting = bool_false;
	handle->iface.cumulative_angle = 0;
	enseq_main_region_OFF_default(handle);
}

static void effect_main_region_REORIENT_SEQUENCE_r1_REORIENT_lr0_lr0(Robot_template* handle)
{
	turn_kobuki(handle->iface.cumulative_angle > 0);
	print_state(REORIENT);
	handle->iface.angle = read_gyro();
	print_angle(fmod((handle->iface.angle + handle->iface.cumulative_angle),360));
}

static void effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_lr0_lr0(Robot_template* handle)
{
	drive_kobuki(bool_true);
	print_state(DRIVING);
	handle->iface.distance = update_dist(handle->iface.distance, handle->iface.prev_encoder, bool_true);
	handle->iface.prev_encoder = read_encoder();
	print_dist(handle->iface.distance);
}

static void effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0(Robot_template* handle)
{
	exseq_main_region_REORIENT_SEQUENCE_r1_DRIVING(handle);
	handle->iface.distance = 0;
	handle->iface.angle = 0;
	start_gyro();
	handle->iface.reorienting = bool_true;
	enseq_main_region_REORIENT_SEQUENCE_r1_REORIENT_default(handle);
}

/* 'default' enter sequence for state OFF */
static void enseq_main_region_OFF_default(Robot_template* handle)
{
	/* 'default' enter sequence for state OFF */
	handle->stateConfVector[0] = Robot_template_main_region_OFF;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state ON */
static void enseq_main_region_ON_default(Robot_template* handle)
{
	/* 'default' enter sequence for state ON */
	enseq_main_region_ON_r1_default(handle);
}

/* 'default' enter sequence for state DRIVING */
static void enseq_main_region_ON_r1_DRIVING_default(Robot_template* handle)
{
	/* 'default' enter sequence for state DRIVING */
	handle->stateConfVector[0] = Robot_template_main_region_ON_r1_DRIVING;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state TURNING */
static void enseq_main_region_ON_r1_TURNING_default(Robot_template* handle)
{
	/* 'default' enter sequence for state TURNING */
	handle->stateConfVector[0] = Robot_template_main_region_ON_r1_TURNING;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state AVOID */
static void enseq_main_region_AVOID_default(Robot_template* handle)
{
	/* 'default' enter sequence for state AVOID */
	enseq_main_region_AVOID_r1_default(handle);
}

/* 'default' enter sequence for state BACKUP */
static void enseq_main_region_AVOID_r1_BACKUP_default(Robot_template* handle)
{
	/* 'default' enter sequence for state BACKUP */
	handle->stateConfVector[0] = Robot_template_main_region_AVOID_r1_BACKUP;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state TURNL */
static void enseq_main_region_AVOID_r1_TURNL_default(Robot_template* handle)
{
	/* 'default' enter sequence for state TURNL */
	handle->stateConfVector[0] = Robot_template_main_region_AVOID_r1_TURNL;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state TURNR */
static void enseq_main_region_AVOID_r1_TURNR_default(Robot_template* handle)
{
	/* 'default' enter sequence for state TURNR */
	handle->stateConfVector[0] = Robot_template_main_region_AVOID_r1_TURNR;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state REORIENT_SEQUENCE */
static void enseq_main_region_REORIENT_SEQUENCE_default(Robot_template* handle)
{
	/* 'default' enter sequence for state REORIENT_SEQUENCE */
	enseq_main_region_REORIENT_SEQUENCE_r1_default(handle);
}

/* 'default' enter sequence for state REORIENT */
static void enseq_main_region_REORIENT_SEQUENCE_r1_REORIENT_default(Robot_template* handle)
{
	/* 'default' enter sequence for state REORIENT */
	handle->stateConfVector[0] = Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for state DRIVING */
static void enseq_main_region_REORIENT_SEQUENCE_r1_DRIVING_default(Robot_template* handle)
{
	/* 'default' enter sequence for state DRIVING */
	handle->stateConfVector[0] = Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING;
	handle->stateConfVectorPosition = 0;
}

/* 'default' enter sequence for region main region */
static void enseq_main_region_default(Robot_template* handle)
{
	/* 'default' enter sequence for region main region */
	react_main_region__entry_Default(handle);
}

/* 'default' enter sequence for region r1 */
static void enseq_main_region_ON_r1_default(Robot_template* handle)
{
	/* 'default' enter sequence for region r1 */
	react_main_region_ON_r1__entry_Default(handle);
}

/* 'default' enter sequence for region r1 */
static void enseq_main_region_AVOID_r1_default(Robot_template* handle)
{
	/* 'default' enter sequence for region r1 */
	react_main_region_AVOID_r1__entry_Default(handle);
}

/* 'default' enter sequence for region r1 */
static void enseq_main_region_REORIENT_SEQUENCE_r1_default(Robot_template* handle)
{
	/* 'default' enter sequence for region r1 */
	react_main_region_REORIENT_SEQUENCE_r1__entry_Default(handle);
}

/* Default exit sequence for state OFF */
static void exseq_main_region_OFF(Robot_template* handle)
{
	/* Default exit sequence for state OFF */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state ON */
static void exseq_main_region_ON(Robot_template* handle)
{
	/* Default exit sequence for state ON */
	exseq_main_region_ON_r1(handle);
}

/* Default exit sequence for state DRIVING */
static void exseq_main_region_ON_r1_DRIVING(Robot_template* handle)
{
	/* Default exit sequence for state DRIVING */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state TURNING */
static void exseq_main_region_ON_r1_TURNING(Robot_template* handle)
{
	/* Default exit sequence for state TURNING */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state AVOID */
static void exseq_main_region_AVOID(Robot_template* handle)
{
	/* Default exit sequence for state AVOID */
	exseq_main_region_AVOID_r1(handle);
}

/* Default exit sequence for state BACKUP */
static void exseq_main_region_AVOID_r1_BACKUP(Robot_template* handle)
{
	/* Default exit sequence for state BACKUP */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state TURNL */
static void exseq_main_region_AVOID_r1_TURNL(Robot_template* handle)
{
	/* Default exit sequence for state TURNL */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state TURNR */
static void exseq_main_region_AVOID_r1_TURNR(Robot_template* handle)
{
	/* Default exit sequence for state TURNR */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state REORIENT_SEQUENCE */
static void exseq_main_region_REORIENT_SEQUENCE(Robot_template* handle)
{
	/* Default exit sequence for state REORIENT_SEQUENCE */
	exseq_main_region_REORIENT_SEQUENCE_r1(handle);
}

/* Default exit sequence for state REORIENT */
static void exseq_main_region_REORIENT_SEQUENCE_r1_REORIENT(Robot_template* handle)
{
	/* Default exit sequence for state REORIENT */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for state DRIVING */
static void exseq_main_region_REORIENT_SEQUENCE_r1_DRIVING(Robot_template* handle)
{
	/* Default exit sequence for state DRIVING */
	handle->stateConfVector[0] = Robot_template_last_state;
	handle->stateConfVectorPosition = 0;
}

/* Default exit sequence for region main region */
static void exseq_main_region(Robot_template* handle)
{
	/* Default exit sequence for region main region */
	/* Handle exit of all possible states (of robot_template.main_region) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Robot_template_main_region_OFF :
		{
			exseq_main_region_OFF(handle);
			break;
		}
		case Robot_template_main_region_ON_r1_DRIVING :
		{
			exseq_main_region_ON_r1_DRIVING(handle);
			break;
		}
		case Robot_template_main_region_ON_r1_TURNING :
		{
			exseq_main_region_ON_r1_TURNING(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_BACKUP :
		{
			exseq_main_region_AVOID_r1_BACKUP(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNL :
		{
			exseq_main_region_AVOID_r1_TURNL(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNR :
		{
			exseq_main_region_AVOID_r1_TURNR(handle);
			break;
		}
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT :
		{
			exseq_main_region_REORIENT_SEQUENCE_r1_REORIENT(handle);
			break;
		}
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING :
		{
			exseq_main_region_REORIENT_SEQUENCE_r1_DRIVING(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for region r1 */
static void exseq_main_region_ON_r1(Robot_template* handle)
{
	/* Default exit sequence for region r1 */
	/* Handle exit of all possible states (of robot_template.main_region.ON.r1) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Robot_template_main_region_ON_r1_DRIVING :
		{
			exseq_main_region_ON_r1_DRIVING(handle);
			break;
		}
		case Robot_template_main_region_ON_r1_TURNING :
		{
			exseq_main_region_ON_r1_TURNING(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for region r1 */
static void exseq_main_region_AVOID_r1(Robot_template* handle)
{
	/* Default exit sequence for region r1 */
	/* Handle exit of all possible states (of robot_template.main_region.AVOID.r1) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Robot_template_main_region_AVOID_r1_BACKUP :
		{
			exseq_main_region_AVOID_r1_BACKUP(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNL :
		{
			exseq_main_region_AVOID_r1_TURNL(handle);
			break;
		}
		case Robot_template_main_region_AVOID_r1_TURNR :
		{
			exseq_main_region_AVOID_r1_TURNR(handle);
			break;
		}
		default: break;
	}
}

/* Default exit sequence for region r1 */
static void exseq_main_region_REORIENT_SEQUENCE_r1(Robot_template* handle)
{
	/* Default exit sequence for region r1 */
	/* Handle exit of all possible states (of robot_template.main_region.REORIENT_SEQUENCE.r1) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT :
		{
			exseq_main_region_REORIENT_SEQUENCE_r1_REORIENT(handle);
			break;
		}
		case Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING :
		{
			exseq_main_region_REORIENT_SEQUENCE_r1_DRIVING(handle);
			break;
		}
		default: break;
	}
}

/* The reactions of state OFF. */
static void react_main_region_OFF(Robot_template* handle)
{
	/* The reactions of state OFF. */
	if (check_main_region_OFF_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_OFF_tr0(handle);
	}  else
	{
		effect_main_region_OFF_lr0_lr0(handle);
	}
}

/* The reactions of state DRIVING. */
static void react_main_region_ON_r1_DRIVING(Robot_template* handle)
{
	/* The reactions of state DRIVING. */
	if (check_main_region_ON_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_ON_tr0(handle);
	}  else
	{
		if (check_main_region_ON_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_ON_tr1(handle);
		}  else
		{
			if (check_main_region_ON_r1_DRIVING_tr0_tr0(handle) == bool_true)
			{ 
				effect_main_region_ON_r1_DRIVING_tr0(handle);
			}  else
			{
				effect_main_region_ON_r1_DRIVING_lr0_lr0(handle);
			}
		}
	}
}

/* The reactions of state TURNING. */
static void react_main_region_ON_r1_TURNING(Robot_template* handle)
{
	/* The reactions of state TURNING. */
	if (check_main_region_ON_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_ON_tr0(handle);
	}  else
	{
		if (check_main_region_ON_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_ON_tr1(handle);
		}  else
		{
			if (check_main_region_ON_r1_TURNING_tr0_tr0(handle) == bool_true)
			{ 
				effect_main_region_ON_r1_TURNING_tr0(handle);
			}  else
			{
				effect_main_region_ON_r1_TURNING_lr0_lr0(handle);
			}
		}
	}
}

/* The reactions of state BACKUP. */
static void react_main_region_AVOID_r1_BACKUP(Robot_template* handle)
{
	/* The reactions of state BACKUP. */
	if (check_main_region_AVOID_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_AVOID_tr0(handle);
	}  else
	{
		if (check_main_region_AVOID_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_AVOID_tr1(handle);
		}  else
		{
			if (check_main_region_AVOID_r1_BACKUP_tr0_tr0(handle) == bool_true)
			{ 
				effect_main_region_AVOID_r1_BACKUP_tr0(handle);
			}  else
			{
				if (check_main_region_AVOID_r1_BACKUP_tr1_tr1(handle) == bool_true)
				{ 
					effect_main_region_AVOID_r1_BACKUP_tr1(handle);
				}  else
				{
					effect_main_region_AVOID_r1_BACKUP_lr0_lr0(handle);
				}
			}
		}
	}
}

/* The reactions of state TURNL. */
static void react_main_region_AVOID_r1_TURNL(Robot_template* handle)
{
	/* The reactions of state TURNL. */
	if (check_main_region_AVOID_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_AVOID_tr0(handle);
	}  else
	{
		if (check_main_region_AVOID_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_AVOID_tr1(handle);
		}  else
		{
			effect_main_region_AVOID_r1_TURNL_lr0_lr0(handle);
		}
	}
}

/* The reactions of state TURNR. */
static void react_main_region_AVOID_r1_TURNR(Robot_template* handle)
{
	/* The reactions of state TURNR. */
	if (check_main_region_AVOID_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_AVOID_tr0(handle);
	}  else
	{
		if (check_main_region_AVOID_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_AVOID_tr1(handle);
		}  else
		{
			effect_main_region_AVOID_r1_TURNR_lr0_lr0(handle);
		}
	}
}

/* The reactions of state REORIENT. */
static void react_main_region_REORIENT_SEQUENCE_r1_REORIENT(Robot_template* handle)
{
	/* The reactions of state REORIENT. */
	if (check_main_region_REORIENT_SEQUENCE_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_REORIENT_SEQUENCE_tr0(handle);
	}  else
	{
		if (check_main_region_REORIENT_SEQUENCE_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_REORIENT_SEQUENCE_tr1(handle);
		}  else
		{
			if (check_main_region_REORIENT_SEQUENCE_tr2_tr2(handle) == bool_true)
			{ 
				effect_main_region_REORIENT_SEQUENCE_tr2(handle);
			}  else
			{
				effect_main_region_REORIENT_SEQUENCE_r1_REORIENT_lr0_lr0(handle);
			}
		}
	}
}

/* The reactions of state DRIVING. */
static void react_main_region_REORIENT_SEQUENCE_r1_DRIVING(Robot_template* handle)
{
	/* The reactions of state DRIVING. */
	if (check_main_region_REORIENT_SEQUENCE_tr0_tr0(handle) == bool_true)
	{ 
		effect_main_region_REORIENT_SEQUENCE_tr0(handle);
	}  else
	{
		if (check_main_region_REORIENT_SEQUENCE_tr1_tr1(handle) == bool_true)
		{ 
			effect_main_region_REORIENT_SEQUENCE_tr1(handle);
		}  else
		{
			if (check_main_region_REORIENT_SEQUENCE_tr2_tr2(handle) == bool_true)
			{ 
				effect_main_region_REORIENT_SEQUENCE_tr2(handle);
			}  else
			{
				if (check_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0_tr0(handle) == bool_true)
				{ 
					effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_tr0(handle);
				}  else
				{
					effect_main_region_REORIENT_SEQUENCE_r1_DRIVING_lr0_lr0(handle);
				}
			}
		}
	}
}

/* Default react sequence for initial entry  */
static void react_main_region__entry_Default(Robot_template* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_OFF_default(handle);
}

/* Default react sequence for initial entry  */
static void react_main_region_ON_r1__entry_Default(Robot_template* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_ON_r1_DRIVING_default(handle);
}

/* Default react sequence for initial entry  */
static void react_main_region_AVOID_r1__entry_Default(Robot_template* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_AVOID_r1_BACKUP_default(handle);
}

/* Default react sequence for initial entry  */
static void react_main_region_REORIENT_SEQUENCE_r1__entry_Default(Robot_template* handle)
{
	/* Default react sequence for initial entry  */
	enseq_main_region_REORIENT_SEQUENCE_r1_DRIVING_default(handle);
}


