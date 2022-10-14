

#ifndef MBED_STEPPER_H
#define MBED_STEPPER_H
#endif

#include "mbed.h"

class stepper
{
public:
    stepper(PinName _en, PinName ms1, PinName ms2, PinName ms3, PinName _stepPin, PinName _dir);
    void step(int edge);
    void direction(int direction);
    void microstep(int ratio);
    void enable();
    void disable();

private:
    DigitalOut en;
    DigitalOut dir;
    BusOut microstepping;
    DigitalOut stepPin;
};
