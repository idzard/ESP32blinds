#include <Arduino.h>
#include <ezButton.h>
#include <FastAccelStepper.h>
#include <ArtnetETH.h>
#include <ESPmDNS.h>

#define stepPinStepper 15
#define dirPinStepper 14
#define enablePinStepper 12

long m1Max = 3200;
long m2Max = 3200;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

ezButton limitSwitch(4);  // create ezButton object that attach to ESP32 pin GPIO4

const IPAddress ip(10, 0, 0, 25);
const IPAddress gateway(10, 0, 0, 1);
const IPAddress subnet_mask(255, 255, 255, 0);

ArtnetReceiver artnet;
uint16_t universe1 = 1; // 0 - 32767
uint8_t net = 0;        // 0 - 127
uint8_t subnet = 0;     // 0 - 15
uint8_t universe2 = 2;  // 0 - 15

unsigned long last = 0;

void callback(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    // will be called on incoming artnet data
    
    Serial.println("");
    Serial.print(data[0]);
    Serial.print("-");
    Serial.print(data[1]);
    Serial.println("");
    
    //combine two bytes into a 16 bit value
    uint16_t m1 = (data[0] * 256) + data[1];
    Serial.print("M1: ");
    Serial.print(m1);
    Serial.println("");

    uint16_t m2 = (data[2] * 256) + data[3];
    Serial.print("M2: ");
    Serial.print(m2);
    Serial.println("");
    
    Serial.println("Art-Net data received:");
    Serial.print(remote.ip);
    Serial.print(":");
    //Serial.print(remote.port);
    //Serial.print(", universe = ");
    //Serial.print(universe1);
    //Serial.print(", size = ");
    //Serial.print(size);
    //Serial.print(") :");
    for (size_t i = 0; i < 4; ++i) {
        Serial.print(data[i]);
        Serial.print(",");
    }
    Serial.println();

    uint16_t m1Remapped = map(m1, 0, 65535, 0, m1Max);
    uint16_t m2Remapped = map(m2, 0, 65535, 0, m2Max);
    Serial.print("moving stepper to: ");
    Serial.print(m1Remapped);
    Serial.println();
    
    stepper->moveTo(m1Remapped);


}



void setup() {
  Serial.begin(115200);
  
  ETH.begin();
  ETH.config(ip, gateway, subnet_mask);

  MDNS.begin("window1"); 
  Serial.println("mDNS responder started");
  
  Serial.println("ETH started");

  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

  engine.init();
  //driver type: 0 = DRIVER_MCPWM_PCNT 
  stepper = engine.stepperConnectToPin(stepPinStepper, 0 );
  
  if (stepper) {
    Serial.println("stepper loaded");
    stepper->setDirectionPin(dirPinStepper);
    stepper->setCurrentPosition(0);
    
    // If auto enable/disable need delays, just add (one or both):
    //stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    //stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setSpeedInHz(20000);
    stepper->setAcceleration(20000);
    //stepper->setJumpStart(100);
    
  }


  artnet.begin();
  artnet.subscribeArtDmxUniverse(net, subnet, universe1, callback);
  Serial.print("Listening");
}

void loop() {
  limitSwitch.loop(); // MUST call the loop() function first
  

  if(limitSwitch.isPressed())
    Serial.println("The button is pressed");

  if(limitSwitch.isReleased())
    Serial.println("The button is released");


  artnet.parse();  // check if artnet packet has come and execute callback function
  
  if (millis() > last + 1000){
      Serial.print(".");
      last = millis();
    }
}