#ifndef PERIPHERAL_INTERFACES_H
#define PERIPHERAL_INTERFACES_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// PERIPH
uint8_t enable_stepper(int stepper_port, int stepper_pin);

uint8_t disable_stepper(int stepper_port, int stepper_pin);

uint8_t set_stepperDirection(int stepper_port, int stepper_pin, bool direction);

uint8_t set_stepperSpeed(int stepper_port, int stepper, int speed);

// probs have to change the return and argument types after looking through documentation
bool get_DIOPinState(int stepper_port, int pin);

int get_ADCValue(int stepper_port, int pin);




#endif // PERIPHERAL_INTERFACES_H
