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

    DNSName = doc["mdnsName"];
    const char* ch_configIPaddress = doc["ipadress"];
    const char* ch_configGateway = doc["gateway"];
    const char* ch_configSubnet = doc["subnet"];

    
    
    ip.fromString(ch_configIPaddress);
    
    gateway.fromString(ch_configGateway);
    
    subnet_mask.fromString(ch_configSubnet);

    // Load stepper direction reversal flags (default to false if not present)
    reverseStepper0 = doc["reverseStepper0"] | false;
    reverseStepper1 = doc["reverseStepper1"] | false;

    // Real world application would store these values in some variables for
    // later use.

    Serial.print("Loaded serverName: ");
    Serial.println(DNSName);
    Serial.print("Loaded ipadress: ");
    Serial.println(ip);
    Serial.print("Stepper0 direction reversed: ");
    Serial.println(reverseStepper0 ? "true" : "false");
    Serial.print("Stepper1 direction reversed: ");
    Serial.println(reverseStepper1 ? "true" : "false");
    return true;
}