#include <mbed.h>
#include "pin_manager.h"
#include "sensors.h"
#include "motors.h"
#include "pid.h"
#include <stepper.h>

#define PI 3.141516

/* Function definitions */
uint8_t initializeRobot(void);
uint8_t initializeUnusedPins(void);
uint8_t initializeAccelerometer(void);
uint8_t initializeMotors(void);
void motorsTimeoutISR(void);

/* Unused pins */
DigitalIn pin_p5(UNUSED_PIN_P5);
DigitalIn pin_p6(UNUSED_PIN_P6);
DigitalIn pin_p7(UNUSED_PIN_P7);
DigitalIn pin_p8(UNUSED_PIN_P8);
DigitalIn pin_p10(UNUSED_PIN_P10);
DigitalIn pin_p17(UNUSED_PIN_P17);

/* UART for remote command & control */
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

/* Motors */
stepper motor_left(PIN_MOTOR_LEFT_ENABLE, PIN_MOTOR_LEFT_MS1, PIN_MOTOR_LEFT_MS2, PIN_MOTOR_LEFT_MS3, PIN_MOTOR_LEFT_STEP, PIN_MOTOR_LEFT_DIR);
stepper motor_right(PIN_MOTOR_RIGHT_ENABLE, PIN_MOTOR_RIGHT_MS1, PIN_MOTOR_RIGHT_MS2, PIN_MOTOR_RIGHT_MS3, PIN_MOTOR_RIGHT_STEP, PIN_MOTOR_RIGHT_DIR);
Timeout motors_timeout;

/* IMU CMPS 12 sensor */
I2C imu_cmps12(IMU_CMPS12_SDA, IMU_CMPS12_SCL);
int imu_cmps12_address = CMPS12_DEFAULT_I2C_ADDRESS;
char imu_cmps12_data[31];

/* Accelerometer */
AnalogIn adxl_x(PIN_ADXL_X);
AnalogIn adxl_y(PIN_ADXL_Y);
AnalogIn adxl_z(PIN_ADXL_Z);

/* Sensor update data */

int main()
{

  printf("Mbed OS version %d.%d.%d\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

  initializeRobot();

  // motors_timeout.attach(&motorsTimeoutISR, 2s); // Starting motors 2 seconds after power up
}

/**
 * ISR functions
 * */

void motorsTimeoutISR(void)
{
  // do nothing (yet)
}

/**
 * Initialization functions
 * */

uint8_t initializeRobot(void)
{
  if (!initializeUnusedPins())
    return false;
  if (!initializeAccelerometer())
    return false;
  if (!initializeMotors())
    return false;
  return true;
}

uint8_t initializeMotor(uint8_t motor)
{
  if (motor == MOTOR_LEFT)
  {
    motor_left.microstep(1);
    motor_left.direction(MOTOR_LEFT_DIRECTION_FORWARD);
    motor_left.enable();
    return true;
  }
  if (motor == MOTOR_RIGHT)
  {
    motor_right.microstep(1);
    motor_right.direction(MOTOR_RIGHT_DIRECTION_FORWARD);
    motor_right.enable();

    return true;
  }
  return false;
}

uint8_t initializeMotors(void)
{
  return initializeMotor(MOTOR_LEFT) && initializeMotor(MOTOR_RIGHT);
}

uint8_t initializeAccelerometer(void)
{
  // Nothing to do
  return true;
}

uint8_t initializeUnusedPins(void)
{
  // Nothing to do
  return true;
}