#include <Arduino.h>
#include <ArtnetETH.h>
#include <ArduinoOSCETH.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ESPUI.h>
#include <ezButton.h>
#include <FastAccelStepper.h>
#include <MycilaWebSerial.h>
#include <Preferences.h>
#include "configLoader.h"
#include "stepper_interface.h"
#include "blinds_core.h"
#include "webui.h"
#include "ota.h"
#include "webserial.h"

WebSerial webSerial;

ezButton button1(4);
ezButton button2(35);


long m1Max = 3200;
long m2Max = 3200;




Preferences preferences;

ArtnetReceiver artnet;
uint16_t universe1 = 1; // 0 - 32767
uint8_t net = 0;        // 0 - 127
uint8_t subnet = 0;     // 0 - 15
uint8_t universe2 = 2;  // 0 - 15

unsigned long lastHeartbeat = 0;


void onOSCReceivedBottomScreenPosition(OscMessage& m) {
    webSerial.printf("received bottom position: %f", m.arg<float>(0));
    moveScreenSafelyFromNormalizedPosition(0, float(m.arg<float>(0)));
}
void onOSCReceivedTopScreenPosition(OscMessage& m) {
    webSerial.printf("received top position: %f", m.arg<float>(0));
    moveScreenSafelyFromNormalizedPosition(1, float(m.arg<float>(0)));
}


void setup() {
  Serial.begin(115200);
  
  if (!loadConfiguration()){
    Serial.println("config loading fucked");
  } else{
    Serial.println("config loaded");
  };
  
  button1.setDebounceTime(50); // set debounce time to 50 milliseconds
  button2.setDebounceTime(50); // set debounce time to 50 milliseconds


  

  // Open Preferences with 'blinds' namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  preferences.begin("blinds", false);
  
  ETH.begin();
  ETH.config(ip, gateway, subnet_mask);
  MDNS.begin(DNSName); 
  
  setupOTA();

  loadStoredStepperValues();
  initializeSteppers();

  setupWebSerialCommands();
 



  setupUI();
  ESPUI.setVerbosity(Verbosity::VerboseJSON);

  // Create a static buffer for the webpage title
  static char titleBuffer[64];
  strcpy(titleBuffer, DNSName);
  strcat(titleBuffer, " Control");
  ESPUI.begin(titleBuffer);
  webSerial.begin(ESPUI.WebServer()); // Initialize WebSerial with ESPUI's server
  

  OscEther.subscribe(7000, "/bottomscreen/position", onOSCReceivedBottomScreenPosition);
  OscEther.subscribe(7000, "/topscreen/position", onOSCReceivedTopScreenPosition);

  //artnet.begin();
  //artnet.subscribeArtDmxUniverse(net, subnet, universe1, onArtnetReceive);

  
  //startHomingSteppers(true);
}



void loop() {
  button1.loop();
  button2.loop();
  if(button1.isPressed()){
    onLimitSwitchPressed(1);
  }
  if(button1.isReleased()){
    onLimitSwitchReleased(1);
  }
  if(button2.isPressed()){
    onLimitSwitchPressed(2);
  }
  if(button2.isReleased()){
    onLimitSwitchReleased(2);
  }

  handleOTA();
  OscEther.update();
  //artnet.parse();  // check if artnet packet has come and execute callback function

}


void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    // will be called on incoming artnet data
    
    //combine two bytes into a 16 bit value
    uint16_t m1 = (data[0] * 256) + data[1];
    //webSerial.print("M1: %hi", m1);
  
    uint16_t m2 = (data[2] * 256) + data[3];
    //webSerial.print("M2: %hi", m2);

    if (calibratedStepper[0]){
      long remappedPosStepper0 = (long)(((long long)m1 * maxPositionStepper[0]) / 65535LL);

      webSerial.printf(">>> moving stepper0 to: %li",remappedPosStepper0 );

      stepper[0]->moveTo(remappedPosStepper0);
    } else{
      //webSerial.println("> incoming artnet but stepper0 not calibrated");
    }
    if (calibratedStepper[1]){
      long remappedPosStepper1 = (long)(((long long)m1 * maxPositionStepper[1]) / 65535LL);
      //webSerial.print(">>> moving stepper0 to: %li",remappedPosStepper1 );
      stepper[1]->moveTo(remappedPosStepper1);
    } else{
      //webSerial.println("> incoming artnet but stepper1 not calibrated");
    }
    
}



