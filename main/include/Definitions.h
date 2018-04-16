// AHR AIR HOCKEY ROBOT PROJECT

#include <stdint.h>

// Variable definitions

// Log and Timer variables
extern long loop_counter;
extern long timer_old;
extern long timer_packet_old;
extern uint64_t timer_value;
extern int debug_counter;

extern uint64_t micros_old;

// We have 2 axis => 2 motor controls 0=X axis   1=Y axis  (Y AXIS HAS 2 MOTORS Left and Right)
extern int16_t speed_m[2];           // Actual speed of motors
extern uint8_t dir_m[2];             // Actual direction of steppers motors

extern uint16_t counter_m[2];        // counters for periods
extern uint16_t period_m[2][8];      // Eight subperiods
extern uint8_t period_m_index[2];    // index for subperiods

// kinematic variables
// position, speed and acceleration are in step units
extern volatile int16_t position_x;  // This variables are modified inside the Timer interrupts
extern volatile int16_t position_y;

extern int16_t speed_x;
extern int16_t speed_y;
extern int16_t max_speed_x;
extern int16_t max_speed_y;

extern int8_t dir_x;     //(dir=1 positive, dir=-1 negative)
extern int8_t dir_y;
extern int16_t target_position_x;
extern int16_t target_position_y;
extern int16_t target_speed_x;
extern int16_t target_speed_y;
extern int16_t max_acceleration_x = MAX_ACCEL_X;  // default maximun acceleration
extern int16_t max_acceleration_y = MAX_ACCEL_Y;
extern int16_t acceleration_x = MAX_ACCEL_X;
extern int16_t acceleration_y = MAX_ACCEL_Y;
extern int16_t accel_ramp = ACCEL_RAMP_MIN;

extern int16_t pos_stop_x;
extern int16_t pos_stop_y;

extern uint16_t com_pos_x;
extern uint16_t com_pos_y;
extern uint16_t com_speed_x;
extern uint16_t com_speed_y;
extern uint16_t target_x_mm;
extern uint16_t target_y_mm;
extern int16_t user_speed_x;
extern int16_t user_speed_y;
extern int16_t filt_user_speed_x;
extern int16_t filt_user_speed_y;


int16_t myAbs(int16_t param);
int sign(int val);
int32_t constrain(int64_t val,int32_t minVal,int32_t maxVal);



