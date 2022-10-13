#include <mbed.h>
#include "pin_manager.h"
#include "sensors.h"
#include "motors.h"
#include "pid.h"

#define PI 3.141516

/* UART for remote command & control */
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

/* IMU CMPS 12 sensor */
I2C imu_cmps12(IMU_CMPS12_SDA, IMU_CMPS12_SCL);
int imu_cmps12_address = CMPS12_DEFAULT_I2C_ADDRESS;
char imu_cmps12_data[31];

int main()
{

  printf("Mbed OS version %d.%d.%d\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

  initializeUnusedPins();
  initializeAccelerometer();
  initializeMotors();

  while (1)
  {
    // put your main code here, to run repeatedly:
  }
}

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
    DigitalOut motor_left_enable(PIN_MOTOR_LEFT_ENABLE);
    DigitalOut motor_left_ms1(PIN_MOTOR_LEFT_MS1);
    DigitalOut motor_left_ms2(PIN_MOTOR_LEFT_MS2);
    DigitalOut motor_left_ms3(PIN_MOTOR_LEFT_MS3);
    DigitalOut motor_left_step(PIN_MOTOR_LEFT_STEP);
    DigitalOut motor_left_dir(PIN_MOTOR_LEFT_DIR);
    return true;
  }
  if (motor == MOTOR_RIGHT)
  {
    DigitalOut motor_right_enable(PIN_MOTOR_RIGHT_ENABLE);
    DigitalOut motor_right_ms1(PIN_MOTOR_RIGHT_MS1);
    DigitalOut motor_right_ms2(PIN_MOTOR_RIGHT_MS2);
    DigitalOut motor_right_ms3(PIN_MOTOR_RIGHT_MS3);
    DigitalOut motor_right_step(PIN_MOTOR_RIGHT_STEP);
    DigitalOut motor_right_dir(PIN_MOTOR_RIGHT_DIR);
    return true;
  }
  return false;
}

uint8_t initializeMotors()
{
  return initializeMotor(MOTOR_LEFT) && initializeMotor(MOTOR_RIGHT);
}

uint8_t initializeAccelerometer(void)
{
  AnalogIn adxl_x(PIN_ADXL_X);
  AnalogIn adxl_y(PIN_ADXL_Y);
  AnalogIn adxl_z(PIN_ADXL_Z);
  return true;
}

uint8_t initializeUnusedPins(void)
{
  DigitalIn pin_p5(UNUSED_PIN_P5);
  DigitalIn pin_p6(UNUSED_PIN_P6);
  DigitalIn pin_p7(UNUSED_PIN_P7);
  DigitalIn pin_p8(UNUSED_PIN_P8);
  DigitalIn pin_p10(UNUSED_PIN_P10);
  DigitalIn pin_p17(UNUSED_PIN_P17);
  return true;
}