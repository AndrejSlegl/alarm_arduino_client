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
      #ifdef DEBUG
      Serial.println("WiFi shield not present");
      #endif
      
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
        #ifdef DEBUG
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(_ssid);
        #endif
        
        int status = WiFi.begin(_ssid, _pass);
    
        if (status == WL_CONNECTED) {
          //Serial.println("Connection established.");
          return true;
        }

        #ifdef DEBUG
        Serial.println(getStatusString(status));
        #endif
      }
    
      return false;
  }

  WiFi.disconnect();
  return false;
}
