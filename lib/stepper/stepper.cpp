#include "stepper.h"
#include "mbed.h"

stepper::stepper(PinName _en, PinName ms1, PinName ms2, PinName ms3, PinName _stepPin, PinName _dir) : en(_en),
                                                                                                      dir(_dir),
                                                                                                      microstepping(ms1, ms2, ms3),
                                                                                                      stepPin(_stepPin)
{

}

void stepper::step(int edge)
{
    stepPin = edge;
}

void stepper::direction(int direction)
{
    dir = direction;
}

void stepper::microstep(int ratio)
{
    /* From A4988 Microstepping Resolution Truth Table */
    switch (ratio)
    {
        case 1:
            microstepping = 0b000;
            break;
        case 2:
            microstepping = 0b001;
            break; 
        case 4:
            microstepping = 0b010;
            break; 
        case 8:
            microstepping = 0b011;
            break; 
        case 16:
            microstepping = 0b111;
            break; 
        default:
            break;  
    }
}

void stepper::enable()
{
    en = 0;
}

void stepper::disable()
{
    en = 1;
}