/**
 * Pin Manager
*/

/* Left Motor */
#define PIN_MOTOR_LEFT_ENABLE p26
#define PIN_MOTOR_LEFT_MS1 p25
#define PIN_MOTOR_LEFT_MS2 p24
#define PIN_MOTOR_LEFT_MS3 p23
#define PIN_MOTOR_LEFT_STEP p22
#define PIN_MOTOR_LEFT_DIR p21

/* Right Motor */
#define PIN_MOTOR_RIGHT_ENABLE p16
#define PIN_MOTOR_RIGHT_MS1 p15
#define PIN_MOTOR_RIGHT_MS2 p14
#define PIN_MOTOR_RIGHT_MS3 p13
#define PIN_MOTOR_RIGHT_STEP p12
#define PIN_MOTOR_RIGHT_DIR p11

/* IMU CMPS12 */
#define IMU_CMPS12_SDA p28
#define IMU_CMPS12_SCL p27

/* Accelerometer */
#define PIN_ADXL_X p18
#define PIN_ADXL_Y p19
#define PIN_ADXL_Z p20

/* UART */
#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX

/* Unused pins must be set as Inputs (good practice) */
#define UNUSED_PIN_P5 p5
#define UNUSED_PIN_P6 p6
#define UNUSED_PIN_P7 p7
#define UNUSED_PIN_P8 p8
#define UNUSED_PIN_P10 p10
#define UNUSED_PIN_P17 p17