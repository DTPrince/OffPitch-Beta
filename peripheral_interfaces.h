#ifndef PERIPHERAL_INTERFACES_H
#define PERIPHERAL_INTERFACES_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// PERIPH
uint8_t enable_stepper(uint8_t stepper);

uint8_t disable_stepper(uint8_t stepper);

uint8_t set_stepperDirection(uint8_t stepper, bool direction);

uint8_t set_stepperSpeed(uint8_t stepper, uint8_t speed);

// probs have to change the return and argument types after looking through documentation
bool get_DIOPinState(uint8_t pin);

int get_ADCValue(uint8_t pin);




#endif // PERIPHERAL_INTERFACES_H
