#ifndef WEBSERIAL_H
#define WEBSERIAL_H

#include <Arduino.h>
#include <MycilaWebSerial.h>

// Function Prototypes
void setupWebSerialCommands();
void sendStatus();
void testHigh();
void testLow();

#endif // WEBSERIAL_H