#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

uint8_t halt();

uint8_t burpLastPacket();

uint8_t moveVSlot_HAB();

uint8_t moveVSlot_SPC();

uint8_t moveVSlot_CEN();

uint8_t clampExperiment();

uint8_t releaseExperiment();

uint8_t open_HABDoor();

uint8_t open_SPCDoor();

uint8_t close_HABDoor();

uint8_t close_SPCDoor();

bool get_tableCapSense();

bool get_plateCapSense();

int get_tableForceSense();

bool get_VSlot_top_HEState();

bool get_VSlot_bot_HEState();

bool get_HABDoor_HEState();

bool get_SPCDoor_HEState();

bool get_HABHinge_HEState();

bool get_SPCHinge_HEState();

int get_Temperature();

#endif // COMMAND_FUNCTIONS_H
