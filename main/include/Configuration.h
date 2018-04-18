
// USER CONFIGURATIONS HERE
// ROBOT DIMENSIONS, MAX SPEED, MAX ACCELERATION, CALIBRATION

// THIS VALUES DEPENDS ON THE VOLTAGE, MOTORS, PULLEYS AND ROBOT CONSTRUCTION
// RECOMMENDED VALUES FOR 12V POWER SUPPLY
#define MIN_ACCEL_X 50 //50
#define MAX_ACCEL_X 500   //360 //300//320      // Max motor acceleration in (steps/seg2)/1000
#define MIN_ACCEL_Y 50
#define MAX_ACCEL_Y 100    //140//220
#define MAX_SPEED_X 20000     //4000//max 25000 for 12V   // Max speed in steps/seg
#define MAX_SPEED_Y 4000
#define SPEED_PAINT 500


// This is for the Accel ramp implementation (to smooth the intial acceleration), simplified S-profile
#define ACCEL_RAMP_MIN 150  // The S profile is generated up to this speed
#define ACCEL_RAMP_MAX 4000

// UNCOMMENT THIS LINES TO INVERT MOTORS
#define INVERT_X_AXIS 1
//#define INVERT_Y_AXIS 1  //Y-LEFT
//#define INVERT_Z_AXIS 1  //Y_RIGHT

// Geometric calibration.
//According to datasheet for IGUS ZLW-1040 we get 66 mm/rev
#define X_AXIS_STEPS_PER_UNIT 24.24   // With RPP 3M band and 1/8 microstepping on drivers
#define Y_AXIS_STEPS_PER_UNIT 24.24    // 200*8 = 1600 steps/rev = 1600/66mm = 24.24 //97 (96.96) mit 32

// Absolute Min and Max robot positions in mm (measured from center of robot pusher)
#define ROBOT_MIN_X 0
#define ROBOT_MIN_Y 0
#define ROBOT_MAX_X 1850
#define ROBOT_MAX_Y 1000

// This is the center of the table. All units in milimeters
#define ROBOT_CENTER_X 500   // Center of robot. The table is 600x1000mm, so center is 300,500
#define ROBOT_CENTER_Y 500

// Initial robot position in mm
// The robot must be at this position at start time
// Default: Centered in X and minimun position in Y
#define ROBOT_INITIAL_POSITION_X 0
#define ROBOT_INITIAL_POSITION_Y 0   // Measured from center of the robot pusher to the table border


#define POSITION_TOLERANCE 5 // 5 steps

#define ENDSTOPX1 GPIO_NUM_32
#define ENDSTOPX2 GPIO_NUM_33

#define ENDSTOPY1 GPIO_NUM_34
#define ENDSTOPY2 GPIO_NUM_35

#define PIN_PULSE_CTR GPIO_NUM_5 // Pulse Counter Pin

// Utils (don´t modify)
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ZERO_SPEED (((uint64_t)1<<32)-1)


#define OLED_CONN 0
#define X_AXIS_ENABLE 1
#define Y_AXIS_ENABLE 0
#define DEBUG_LED_PIN GPIO_NUM_23

#define X_DIR GPIO_NUM_16
#define X_STEP GPIO_NUM_18
#define X_ENABLE GPIO_NUM_15

#define Y_DIR GPIO_NUM_16
#define Y_STEP GPIO_NUM_18
#define Y_ENABLE GPIO_NUM_35


//
#define SERVO_PIN GPIO_NUM_13

// GPIO_NUM_19 and GPIO_NUM_22 are used in OLED communication

#define ABSOLUTE 1
#define INCREMENTAL 0

#define X_BW 1
#define X_FW 0

#define Y_BW 0
#define Y_FW 1

#define I2C_SDA GPIO_NUM_19 //GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

//21SDA y 22SCL
