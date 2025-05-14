#include "configLoader.h"


IPAddress ip;
IPAddress gateway;
IPAddress subnet_mask;
const char* DNSName;
bool reverseStepper0 = false;
bool reverseStepper1 = false;

bool loadConfiguration() {
    Serial.println("config loading");
    if (!SPIFFS.begin(true)) {
        Serial.println("An error has occurred while mounting LittleFS");
        return false;
    }

    File configFile = SPIFFS.open("/config.json");
    if (!configFile) {
        Serial.println("Failed to open config file");
        return false;
    }
    JsonDocument doc;
    auto error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println("Failed to parse config file");
        return false;
    }

    // Make a persistent copy of the string
    static char mdnsNameBuffer[32]; // Buffer to store the mdnsName
    strlcpy(mdnsNameBuffer, doc["mdnsName"] | "window1", sizeof(mdnsNameBuffer));
    DNSName = mdnsNameBuffer;
    const char* ch_configIPaddress = doc["ipadress"];
    const char* ch_configGateway = doc["gateway"];
    const char* ch_configSubnet = doc["subnet"];

    
    
    ip.fromString(ch_configIPaddress);
    
    gateway.fromString(ch_configGateway);
    
    subnet_mask.fromString(ch_configSubnet);

    // Load stepper direction reversal flags (default to false if not present)
    reverseStepper0 = doc["reverseStepper0"] | false;
    reverseStepper1 = doc["reverseStepper1"] | false;


    return true;
}