
#ifndef ROBOT_TEMPLATE_H_
#define ROBOT_TEMPLATE_H_

#include "../src/sc_types.h"
#include "../helper_funcs.h"
#include "../states.h"
		
#ifdef __cplusplus
extern "C" { 
#endif 

/*! \file Header of the state machine 'robot_template'.
*/


/*! Enumeration of all states */ 
typedef enum
{
	Robot_template_last_state,
	Robot_template_main_region_OFF,
	Robot_template_main_region_ON,
	Robot_template_main_region_ON_r1_DRIVING,
	Robot_template_main_region_ON_r1_TURNING,
	Robot_template_main_region_AVOID,
	Robot_template_main_region_AVOID_r1_BACKUP,
	Robot_template_main_region_AVOID_r1_TURNL,
	Robot_template_main_region_AVOID_r1_TURNR,
	Robot_template_main_region_REORIENT_SEQUENCE,
	Robot_template_main_region_REORIENT_SEQUENCE_r1_REORIENT,
	Robot_template_main_region_REORIENT_SEQUENCE_r1_DRIVING
} Robot_templateStates;

/*! Type definition of the data structure for the Robot_templateIface interface scope. */
typedef struct
{
	float distance;
	uint16_t prev_encoder;
	float angle;
	float cumulative_angle;
	sc_boolean turnL;
	sc_boolean turnR;
	sc_boolean reorienting;
} Robot_templateIface;


/*! Define dimension of the state configuration vector for orthogonal states. */
#define ROBOT_TEMPLATE_MAX_ORTHOGONAL_STATES 1


/*! Define indices of states in the StateConfVector */
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_OFF 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON_R1_DRIVING 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_ON_R1_TURNING 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_BACKUP 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_TURNL 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_AVOID_R1_TURNR 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE_R1_REORIENT 0
#define SCVI_ROBOT_TEMPLATE_MAIN_REGION_REORIENT_SEQUENCE_R1_DRIVING 0

/*! 
 * Type definition of the data structure for the Robot_template state machine.
 * This data structure has to be allocated by the client code. 
 */
typedef struct
{
	Robot_templateStates stateConfVector[ROBOT_TEMPLATE_MAX_ORTHOGONAL_STATES];
	sc_ushort stateConfVectorPosition; 
	
	Robot_templateIface iface;
} Robot_template;


/*! Initializes the Robot_template state machine data structures. Must be called before first usage.*/
extern void robot_template_init(Robot_template* handle);

/*! Activates the state machine */
extern void robot_template_enter(Robot_template* handle);

/*! Deactivates the state machine */
extern void robot_template_exit(Robot_template* handle);

/*! Performs a 'run to completion' step. */
extern void robot_template_runCycle(Robot_template* handle);


/*! Gets the value of the variable 'distance' that is defined in the default interface scope. */ 
extern float robot_templateIface_get_distance(const Robot_template* handle);
/*! Sets the value of the variable 'distance' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_distance(Robot_template* handle, float value);
/*! Gets the value of the variable 'prev_encoder' that is defined in the default interface scope. */ 
extern uint16_t robot_templateIface_get_prev_encoder(const Robot_template* handle);
/*! Sets the value of the variable 'prev_encoder' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_prev_encoder(Robot_template* handle, uint16_t value);
/*! Gets the value of the variable 'angle' that is defined in the default interface scope. */ 
extern float robot_templateIface_get_angle(const Robot_template* handle);
/*! Sets the value of the variable 'angle' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_angle(Robot_template* handle, float value);
/*! Gets the value of the variable 'cumulative_angle' that is defined in the default interface scope. */ 
extern float robot_templateIface_get_cumulative_angle(const Robot_template* handle);
/*! Sets the value of the variable 'cumulative_angle' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_cumulative_angle(Robot_template* handle, float value);
/*! Gets the value of the variable 'turnL' that is defined in the default interface scope. */ 
extern sc_boolean robot_templateIface_get_turnL(const Robot_template* handle);
/*! Sets the value of the variable 'turnL' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_turnL(Robot_template* handle, sc_boolean value);
/*! Gets the value of the variable 'turnR' that is defined in the default interface scope. */ 
extern sc_boolean robot_templateIface_get_turnR(const Robot_template* handle);
/*! Sets the value of the variable 'turnR' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_turnR(Robot_template* handle, sc_boolean value);
/*! Gets the value of the variable 'reorienting' that is defined in the default interface scope. */ 
extern sc_boolean robot_templateIface_get_reorienting(const Robot_template* handle);
/*! Sets the value of the variable 'reorienting' that is defined in the default interface scope. */ 
extern void robot_templateIface_set_reorienting(Robot_template* handle, sc_boolean value);

/*!
 * Checks whether the state machine is active (until 2.4.1 this method was used for states).
 * A state machine is active if it was entered. It is inactive if it has not been entered at all or if it has been exited.
 */
extern sc_boolean robot_template_isActive(const Robot_template* handle);

/*!
 * Checks if all active states are final. 
 * If there are no active states then the state machine is considered being inactive. In this case this method returns false.
 */
extern sc_boolean robot_template_isFinal(const Robot_template* handle);

/*! Checks if the specified state is active (until 2.4.1 the used method for states was called isActive()). */
extern sc_boolean robot_template_isStateActive(const Robot_template* handle, Robot_templateStates state);



#ifdef __cplusplus
}
#endif 

#endif /* ROBOT_TEMPLATE_H_ */
