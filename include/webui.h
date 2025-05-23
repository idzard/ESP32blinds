#pragma once

#include <ESPUI.h>

//UI handles



// Function declarations for UI callbacks
void buttonCallback(Control* sender, int type);

void updateCalibrationStatusStepper(uint8_t stepperId);
void updateHomingStatusStepper(uint8_t stepperId);

void updateBottomScreenPositionSlider(uint8_t percentage);
void updateTopScreenPositionSlider(uint8_t percentage);

void setupUI();
