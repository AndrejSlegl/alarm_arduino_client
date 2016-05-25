#include "WiFiPersistentConnector.h"
#include "WiFiUtils.h"

#include <SPI.h>
#include <WiFi101.h>

WiFiPersistentConnector::WiFiPersistentConnector(const char *ssid, const char *pass) {
    _ssid = ssid;
    _pass = pass;
}

bool WiFiPersistentConnector::start() {
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      return false;
    }

    _isStarted = true;
    return true;
}

void WiFiPersistentConnector::stop() {
    _isStarted = false;
}

bool WiFiPersistentConnector::update() {
    if (_isStarted) {
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(_ssid);
        int status = WiFi.begin(_ssid, _pass);
    
        if (status == WL_CONNECTED) {
          //Serial.println("Connection established.");
          return true;
        }
        
        Serial.println(getStatusString(status));
      }
    
      return false;
  }

  WiFi.disconnect();
  return false;
}