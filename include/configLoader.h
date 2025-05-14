#pragma once

#include <ArduinoJson.h>
#include <SPIFFS.h>



extern const char* DNSName;
extern IPAddress ip;
extern IPAddress gateway;
extern IPAddress subnet_mask;
extern bool reverseStepper0;
extern bool reverseStepper1;

bool loadConfiguration();