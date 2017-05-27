#ifndef _SERVOIFACE_H_
#define	_SERVOIFACE_H_


// -----------------------------------------------------------------------------
// PROTOTYPES
// -----------------------------------------------------------------------------
void SetObjective (int32_t goal_position_in_ticks);
void GoToObjective();
int IsGoalReached();
int IsGoalActive();
void CheckForZero();
int ZeroIsFound();
void ZeroTriggered();

void (* hw_set_motor_speed)(int);
int32_t (* hw_get_current_position)();
void (* hw_goal_is_active)();
void (* hw_goal_is_reached)();

void set_hw_set_motor_speed(void (*hw_set_motor_speed_pointer)(int));
int32_t set_hw_get_current_position(int32_t (*hw_get_current_position_pointer)());
void set_hw_goal_is_active(void (*hw_goal_is_active_pointer)());
void set_hw_goal_is_reached(void (*hw_goal_is_reached_pointer)());

void SetMotorSpeed(int speed);
void GoalIsReached();
void GoalIsActive();
int32_t GetCurrentPosition();


#endif	/* _SERVOIFACE_H_ */
