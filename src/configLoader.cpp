#include "configLoader.h"


IPAddress ip;
IPAddress gateway;
IPAddress subnet_mask;
const char* DNSName;

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


    // Real world application would store these values in some variables for
    // later use.

    Serial.print("Loaded serverName: ");
    Serial.println(ip);
    Serial.print("Loaded ipadress: ");
    Serial.println(ip);
    return true;
}