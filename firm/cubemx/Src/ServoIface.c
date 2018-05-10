#include "main.h"
#include "stdint.h"
#include "stm32f0xx_hal.h"

int32_t encoder_0_value=0; // the tick number corresponding to slider's minimum limit
int32_t current_position = 0; // the tick number corresponding to the motor current position

int32_t goal_position; // the tick number that must be reached by the motor
int goal_is_active = 0; // 0 when no goal is set, 1 otherwise
int goal_is_forward = 0; // goal is forward when that encoder value should increase to get closer to the goal

int zero_is_found = 0; // 1 if the correct value for encoder_0_value has been set

// -----------------------------------------------------------------------------
// HARDWARE FUNCTION POINTERS
// -----------------------------------------------------------------------------
void (* hw_set_motor_speed)(int) = NULL;
int32_t (* hw_get_current_position)() = NULL;
void (* hw_goal_is_active)() = NULL;
void (* hw_goal_is_reached)() = NULL;


// -----------------------------------------------------------------------------
// HARDWARE FUNCTION INITIALIZERS
// -----------------------------------------------------------------------------
void set_hw_set_motor_speed(void (*hw_set_motor_speed_pointer)(int))
{
  hw_set_motor_speed = hw_set_motor_speed_pointer;
}

int32_t set_hw_get_current_position(int32_t (*hw_get_current_position_pointer)())
{
  hw_get_current_position = hw_get_current_position_pointer;
}

void set_hw_goal_is_active(void (*hw_goal_is_active_pointer)())
{
  hw_goal_is_active = hw_goal_is_active_pointer;
}

void set_hw_goal_is_reached(void (*hw_goal_is_reached_pointer)())
{
  hw_goal_is_reached = hw_goal_is_reached_pointer;
}


// -----------------------------------------------------------------------------
// HARDWARE FUNCTION BRIDGES
// -----------------------------------------------------------------------------

/**
  * @brief Set the motor speed.
  * @param speed: the new motor speed. If positive the motor will move forward, if negative it will move backward.
  */
void SetMotorSpeed(int speed) {
	hw_set_motor_speed(speed);
}

/**
  * @brief Called when the goal is reached.
  */
void GoalIsReached() {
	hw_goal_is_reached();
}

/**
  * @brief Called when the goal is set.
  */
void GoalIsActive() {
	hw_goal_is_active();
}

/**
  * @brief Return the current motor position. Unit = ticks as read by the encoder.
  */
int32_t GetCurrentPosition() {
	return hw_get_current_position();
}


// -----------------------------------------------------------------------------
// SERVOIFACE LOGIC
// -----------------------------------------------------------------------------

/**
  * @brief Set the new position to be reached.
  * @param goal_position_in_ticks: the new position to be reached. Unit = ticks as read by the encoder (min = 0, max = number of ticks to reach the end of the slider).
  */
void SetObjective (int32_t goal_position_in_ticks)
{
	int32_t goal_position_normalized = encoder_0_value + goal_position_in_ticks;

	if(goal_position_normalized != current_position) {

		if(goal_position_normalized > current_position) {
			goal_is_forward = 1;
		} else {
			goal_is_forward = 0;
		}
	}

	goal_position = goal_position_normalized;
	goal_is_active = 1;
	GoalIsActive();
}

/**
  * @brief Return 1 if the goal is reached, or 0 if not.
  */
int IsGoalReached() {
	int is_goal_reached = 0;

	current_position = GetCurrentPosition();

	if(goal_is_active == 0) {
		is_goal_reached = 1;
	} else {
		if((goal_is_forward == 1 && current_position >= goal_position)
		|| (goal_is_forward == 0 && current_position <= goal_position)) {
			is_goal_reached = 1;
		}
	}

	return is_goal_reached;
}

/**
  * @brief Move the motor to the objective. Must be called after SetObjective()
  */
void GoToObjective() {

	if(goal_is_forward == 1) {
		SetMotorSpeed(600);
	} else {
		SetMotorSpeed(-600);
	}

	while (IsGoalReached() == 0) {

	}

	SetMotorSpeed(0);
	goal_is_active = 0;
	GoalIsReached();
}

/**
  * @brief Return 1 if the goal is active (then the motor is still moving), or 0 if not (then the motor is still).
  */
int IsGoalActive() {
	return goal_is_active;
}

/**
  * @brief Move the motor backward until it trigger the 0 interrupt.
  */
void CheckForZero() {
	zero_is_found = 0;
	SetMotorSpeed(SPEED_INIT);
}

/**
  * @brief Return 1 if the Zero limit has been found, or 0 if not.
  */
int ZeroIsFound() {
	return zero_is_found;
}

/**
  * @brief Should be called when the 0 interrupt is triggered.
  */
void ZeroTriggered() {
	if (zero_is_found == 0) {
		SetMotorSpeed(0);
		encoder_0_value = GetCurrentPosition();
		zero_is_found = 1;
	}
}
