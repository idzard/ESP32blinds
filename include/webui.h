#pragma once

#include <ESPUI.h>

//UI handles



// Function declarations for UI callbacks
void buttonCallback(Control* sender, int type);

void updateCalibrationStatusStepper(uint8_t stepperId);
void updateHomingStatusStepper(uint8_t stepperId);


void setupUI();
