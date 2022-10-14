/* Test motor direction and ratio */
/* Copy Paste in main.cpp */
/* TODO: integrate in unit tests */

int direction = 1;
while (1)
{

    /* Change direction */
    direction = !direction;
    motor_left.direction(direction);
    motor_right.direction(!direction);

    /* Test all ratios */
    int ratio = 1;
    while (ratio <= 16)
    {
        motor_left.microstep(ratio);
        motor_right.microstep(ratio);
        printf("Ratio = 1:%d\r\n", ratio);

        for (int j = 1; j < 100 * ratio; j++)
        {
            motor_left.step(1);
            motor_right.step(1);
            ThisThread::sleep_for(10ms / ratio);
            motor_left.step(0);
            motor_right.step(0);
            ThisThread::sleep_for(10ms / ratio);
        }

        switch (ratio)
        {
        case 1:
            ratio = 2;
            break;
        case 2:
            ratio = 4;
            break;
        case 4:
            ratio = 8;
            break;
        case 8:
            ratio = 16;
            break;
        case 16:
            ratio = 32;
            break;
        }
    }
}
