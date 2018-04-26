#ifndef PERIPHERAL_INTERFACES_H_
#define PERIPHERAL_INTERFACES_H_

#include <stdint.h>
#include <stdbool.h>

void initSettings(void);

uint8_t enable_stepper(int port, int pin);

uint8_t disable_stepper(int port, int pin);

uint8_t disable_PWM(int port, int pin);

uint8_t enable_PWM(int port, int pin);

uint8_t set_stepperDirection(int port, int pin, bool direction);

uint8_t set_stepperSpeed(int port, int pin, int speed);

uint8_t get_DIOPinState(int port, int pin);

uint16_t get_ADCValue(int port, int pin);

#endif // PERIPHERAL_INTERFACES_H_
