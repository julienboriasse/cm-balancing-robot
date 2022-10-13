/**
 * PID
 */ 

/* Loop 1 */
#define KP -30.0
#define KI -0.04
#define KD -0.02
#define K 0.0
#define ROLL_ERROR_SUM_MAX 10000 // Limit integral term

/* Loop 2 */
#define KI_POSITION -0.001
#define KD_POSITION +0.0001
#define ROLL_SP_MAX 10
