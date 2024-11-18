#include <Arduino.h>
#include <ezButton.h>
#include <FastAccelStepper.h>
#include <ArtnetETH.h>
#include <ESPmDNS.h>

#define stepPinStepper1 15
#define dirPinStepper1 14

#define stepPinStepper2 15
#define dirPinStepper2 14

#define DRIVER_MCPWM_PCNT 0


long m1Max = 3200;
long m2Max = 3200;

struct stepper_config_s {
  uint8_t step;
  uint8_t direction;
  uint32_t speed;
  uint32_t acceleration;
};

const struct stepper_config_s stepper_config[2] = {
    {
      step : stepPinStepper1,
      direction : dirPinStepper1,
      speed: 30000,
      acceleration: 20000
    },
    {
      step : stepPinStepper2,
      direction : dirPinStepper2,
      speed: 30000,
      acceleration: 20000
    }};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper[2];

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

void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote);
void homeSteppers();


void setup() {
  Serial.begin(115200);
  
  ETH.begin();
  ETH.config(ip, gateway, subnet_mask);

  MDNS.begin("window1"); 
  
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

  engine.init();
  for (uint8_t i = 0; i < 2; i++) {
    FastAccelStepper *s = NULL;
    const struct stepper_config_s *config = &stepper_config[i];
    if (config->step != PIN_UNDEFINED) {
      s = engine.stepperConnectToPin(config->step, DRIVER_MCPWM_PCNT);
      if (s) {
        s->setDirectionPin(config->direction);
        s->setSpeedInHz(config->speed);
        s->setAcceleration(config->acceleration);
      }
    }
    stepper[i] = s;
  }

  artnet.begin();
  artnet.subscribeArtDmxUniverse(net, subnet, universe1, onArtnetReceive);

  //homeSteppers();
}

void loop() {
  limitSwitch.loop();
  
  if(limitSwitch.isPressed())
    stepper[0]->forceStop();
    Serial.println("Stepper0 is homed");
    

  if(limitSwitch.isReleased())
    Serial.println("The button is released");


  artnet.parse();  // check if artnet packet has come and execute callback function
  
  if (millis() > last + 1000){
      Serial.print(".");
      last = millis();
    }
}


void onArtnetReceive(const uint8_t *data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
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
    
    stepper[0]->moveTo(m1Remapped);

}


void homeSteppers() {
  stepper[0]->runForward();
}