#include <mbed.h>
#include "pin_manager.h"
#include "sensors.h"
#include "motors.h"
#include "pid.h"
#include <stepper.h>

#define PI 3.141516
#define ROLL_SAMPLES_N 10

/* Function definitions */
uint8_t initializeRobot(void);
uint8_t initializeMotors(void);
uint8_t initializeMotor(uint8_t motor);
uint8_t initializeUnusedPins(void);
uint8_t initializeAccelerometer(void);
void motorsTimeoutISR(void);
void displayISR(void);
void sensorsISR(void);
void absolutePositionISR(void);

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

/* PID 1 */
int roll = 0;
int roll_samples[ROLL_SAMPLES_N];
int roll_samples_index = 0;
int roll_set_point = 0;
int roll_error = 0;
int roll_error_sum = 0;
int roll_error_previous = 0;
int roll_error_delta = 0;
int u = 0;               // command to be sent to motors
uint32_t delta_t = 1000; // time interval for PID computation
int u_p = 0;
int u_i = 0;
int u_d = 0;

/* PID 2 */
int32_t absolute_position = 0;
int32_t absolute_position_sp = 0;
int32_t absolute_position_delta_error = 0;
int32_t absolute_position_error = 0;
int32_t absolute_position_error_previous = 0;
int32_t absolute_position_error_sum = 0;

/* Accelerometer */
AnalogIn adxl_x(PIN_ADXL_X);
AnalogIn adxl_y(PIN_ADXL_Y);
AnalogIn adxl_z(PIN_ADXL_Z);

/* LED */
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

Ticker display_ticker;
bool display_flag = false;

Ticker sensors_ticker;
bool sensors_flag = false;

bool display_pid_flag = false;

Ticker absolute_position_ticker;

int main()
{

  printf("Mbed OS version %d.%d.%d\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

  initializeRobot();

  display_ticker.attach(&displayISR, 1s);
  sensors_ticker.attach(&sensorsISR, ROLL_PID_DELTA_T_PERIOD);
  absolute_position_ticker.attach(&absolutePositionISR, 200ms);
  motors_timeout.attach(&motorsTimeoutISR, 2s); // Starting motors 2 seconds after power up

  while (1)
  {
    if (sensors_flag)
    {
      imu_cmps12.write(imu_cmps12_address, 0, 31);
      imu_cmps12.read(imu_cmps12_address, imu_cmps12_data, sizeof(imu_cmps12_data));
      int8_t roll_tmp = imu_cmps12_data[5];
      if (roll_tmp > 90)
      {
        roll_tmp = roll_tmp - 255;
      }

      // moving average
      roll_samples[roll_samples_index] = roll_tmp * 100;
      roll_samples_index++;
      if (roll_samples_index > ROLL_SAMPLES_N)
      {
        roll_samples_index = 0;
      }
      int roll_samples_sum = 0;
      for (int i = 0; i < ROLL_SAMPLES_N; i++)
      {
        roll_samples_sum += roll_samples[i];
      }
      roll = roll_samples_sum / ROLL_SAMPLES_N;
      printf("R: %d°\r\n", roll);

      sensors_flag = false;
    }

    if (display_flag)
    {
      /*
      printf("Monitoring: ");
      printf("Roll=%5d\t", roll);
      printf("roll_set_point=%5d\t", roll_set_point);
      printf("roll_error=%5d\t", roll_error);
      printf("roll_error_sum=%5d\t", roll_error_sum);
      printf("roll_error_delta=%5d\t", roll_error_delta);
      printf("u=%5d\t", u);
      printf("absolute_position_error=%5ld\t", absolute_position_error);
      printf("absolute_position_delta_error=%5ld\t", absolute_position_delta_error);
      printf("\r\n");
      */
      display_flag = false;
    }

    if (display_pid_flag)
    {
      printf("PID: %5d %5d %5d, E: %4d\r\n", u_p, u_i, u_d, roll_error);
      display_pid_flag = false;
    }

    if (abs(roll) > 25 * 100)
    {
      printf("Disable motors - Roll exceeds 25°\r\n");
      motor_left.disable();
      motor_right.disable();
    }
  }
}

/**
 * ISR functions
 * */
void absolutePositionISR(void)
{
  /*
  absolute_position_error = absolute_position_sp - absolute_position;
  absolute_position_error_sum = absolute_position_error + absolute_position_error;
  absolute_position_delta_error = absolute_position_error - absolute_position_error_previous;

  roll_set_point = (absolute_position_error * KP_POSITION + absolute_position_error_sum * KI_POSITION + absolute_position_delta_error * KD_POSITION) * 100;
  roll_set_point = roll_set_point > ROLL_SP_MAX ? ROLL_SP_MAX : roll_set_point;
  roll_set_point = roll_set_point < -ROLL_SP_MAX ? -ROLL_SP_MAX : roll_set_point;
  absolute_position_error_previous = absolute_position_error;
  */
}

void motorsTimeoutISR(void)
{
  roll_error = roll_set_point - roll;
  roll_error_sum += roll_error * ROLL_PID_DELTA_T;
  roll_error_delta = (roll_error - roll_error_previous) / ROLL_PID_DELTA_T;
  roll_error_previous = roll_error;

  // Limit error sum
  roll_error_sum = roll_error_sum > ROLL_ERROR_SUM_MAX ? ROLL_ERROR_SUM_MAX : roll_error_sum;
  roll_error_sum = roll_error_sum < -ROLL_ERROR_SUM_MAX ? -ROLL_ERROR_SUM_MAX : roll_error_sum;

  u_p = ROLL_KP * roll_error;
  u_i = ROLL_KI * roll_error_sum;
  u_d = ROLL_KD * roll_error_delta;

  display_pid_flag = true;

  u = ROLL_K + (u_p + u_i + u_d) / 100; // hundred degree per second
  u = u > 20000 ? 20000 : u;
  u = u < -20000 ? -20000 : u;

  int vitesse_deg_s = u;

  if (u != 0)
  {
    led2 = 0;
    motors_timeout.attach_us(motorsTimeoutISR, 562500 / abs(vitesse_deg_s) / 16);

    if (u < 0)
    {
      absolute_position++;
    }
    else if (u > 0)
    {
      absolute_position--;
    }
    motor_right.direction(u < 0);
    motor_left.direction(!(u < 0));

    motor_left.toggleStep();
    motor_right.toggleStep();
  }
  else
  {
    led2 = 1;
    motors_timeout.attach(motorsTimeoutISR, 10ms);
  }
}

void displayISR(void)
{
  display_flag = true;
}

void sensorsISR(void)
{
  sensors_flag = true;
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

uint8_t initializeMotors(void)
{
  return initializeMotor(MOTOR_LEFT) && initializeMotor(MOTOR_RIGHT);
}

uint8_t initializeMotor(uint8_t motor)
{
  if (motor == MOTOR_LEFT)
  {
    motor_left.microstep(16);
    motor_left.direction(MOTOR_LEFT_DIRECTION_FORWARD);
    motor_left.enable();
    return true;
  }
  if (motor == MOTOR_RIGHT)
  {
    motor_right.microstep(16);
    motor_right.direction(MOTOR_RIGHT_DIRECTION_FORWARD);
    motor_right.enable();

    return true;
  }
  return false;
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