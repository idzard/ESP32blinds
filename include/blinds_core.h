#ifndef BLINDS_CORE_H
#define BLINDS_CORE_H

#include <Arduino.h>
#include <ArtnetETH.h> // For ArtDmxMetadata, ArtNetRemoteInfo
#include <ezButton.h> // For ezButton type
#include <Preferences.h> // For Preferences type
#include <MycilaWebSerial.h> // For WebSerial type
class AsyncWebServer;


extern WebSerial webSerial;
extern ezButton button1;
extern ezButton button2;

extern Preferences preferences;
extern AsyncWebServer webserver;

extern ArtnetReceiver artnet;
extern uint16_t universe1;
extern uint8_t net;
extern uint8_t subnet;
extern uint8_t universe2;

extern unsigned long lastHeartbeat;

void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote);
void onLimitSwitchPressed(int buttonId);
void onLimitSwitchReleased(int buttonId);
void sendStatus();
void testHigh();
void testLow();

#endif // BLINDS_CORE_H