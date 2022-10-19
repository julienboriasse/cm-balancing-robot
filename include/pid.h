/**
 * PID
 */ 

/* Loop 1 */
#define ROLL_PID_DELTA_T 0.01
#define ROLL_PID_DELTA_T_PERIOD 10ms
#define ROLL_SAMPLES_N 10
#define ROLL_KP -30.0
#define ROLL_KI -0.0
#define ROLL_KD -2.0
#define ROLL_K 0.0
#define ROLL_ERROR_SUM_MAX 100000 // Limit integral term



/* Loop 2 */
#define POSITION_PID_DELTA_T 0.1
#define POSITION_PID_DELTA_T_PERIOD 100ms
#define POSITION_SAMPLES_N 30
#define POSITION_KP -0.1
#define POSITION_KI -0.001
#define POSITION_KD -0.01
#define ROLL_SP_MAX 1000
