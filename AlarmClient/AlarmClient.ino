#include <SPI.h>
#include <WiFi101.h>
#include "WiFiPersistentConnector.h"
#include "ValueChangeEventQueue.h"

//#define DEBUG

const char NEW_LINE[] = "\r\n";
const int lightSensorPin = 0;
const int clientLedPin = 6;
const int movementSensor0Pin = 0;
const int sirenPin = 7;
int sensorReadingPositiveThreshold = 800;
int readingSamplingMillis = 50;
int sirenTimeoutMillis = 5 * 1000 * 60;

const char sectorText[] = "sector0:";
const char rssiText[] = "rssi:";
const char movement0Text[] = "movement0:";
const char statusQueryText[] = "status?";
const char stopSirenText[] = "stopSiren!";
char server[] = "192.168.1.111";
int port = 43254;
WiFiPersistentConnector connector("SLEGL WiFi", "pnr41wlan");
WiFiClient client;
ValueChangeEventQueue lightSensorEventQueue(20);
ValueChangeEventQueue movementSensor0EventQueue(20);
String serverCommand;
bool wifiConnected = false;
bool clientConnected = false;
bool clientConnectionOn = true;

bool prevSensorState = false;
int sensorValueSum = 0;
int sensorValueCount = 0;
int sensorLastSampleMillis = 0;
bool movementSensor0Value = false;
int sirenStartedMillis = 0;
bool sirenActivated = false;

void updateClientConnectionStatus(bool connected) {
  if (clientConnected == connected)
    return;

  clientConnected = connected;
  
  digitalWrite(clientLedPin, connected ? HIGH : LOW);
  
#ifdef DEBUG
  Serial.println(connected ? "Connected to server" : "Disconnected from server");
#endif
}

void keepClientConnected() {
  if (wifiConnected) {
    bool connected = client.connected();
    
    updateClientConnectionStatus(connected);
    if (!connected && clientConnectionOn) {
      connected = client.connect(server, port);
      updateClientConnectionStatus(connected);
    }

    if (connected && !clientConnectionOn)
      client.stop();
  }
  else {
    client.stop();
    updateClientConnectionStatus(false);
  }
}

void startSiren() {
  if (sirenActivated) {
    digitalWrite(sirenPin, HIGH);
    sirenStartedMillis = millis();
  }
}

void stopSiren() {
  digitalWrite(sirenPin, LOW);
  sirenStartedMillis = -1;
}

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  //while (!Serial);
#else
  clientConnectionOn = true;
#endif

  pinMode(movementSensor0Pin, INPUT_PULLUP);
  pinMode(clientLedPin, OUTPUT);
  pinMode(sirenPin, OUTPUT);

  stopSiren();
  
  connector.start();
}

void loop() {
  connector.update();

  bool wifiCon = WiFi.status() == WL_CONNECTED;
  if(wifiCon != wifiConnected) {
    wifiConnected = wifiCon;

#ifdef DEBUG
    Serial.println(wifiConnected ? "WiFi Connected" : "WiFi Disconnected");
    if (wifiConnected) {
      IPAddress ip = WiFi.localIP();
      Serial.print("IP Address: ");
      Serial.println(ip);
    }
#endif
  }

  while (client.available() && serverCommand.lastIndexOf(NEW_LINE) < 0) {
    char c = client.read();
    serverCommand += c;
  }

  keepClientConnected();

#ifdef DEBUG
  String command;
  while(Serial.available()) {
    command += (char)Serial.read();
  }

  if (command.length() > 0) {
    if (command == "client on")
      clientConnectionOn = true;
    else if (command == "client off")
      clientConnectionOn = false;
    else if (command == "wifi on")
      connector.start();
    else if (command == "wifi off")
      connector.stop();
  }
#endif

  int sensorValue = analogRead(lightSensorPin);
  int time = millis();

  sensorValueSum += sensorValue;
  sensorValueCount++;
  
  if(time - sensorLastSampleMillis >= readingSamplingMillis) {
    sensorValue = sensorValueSum / sensorValueCount;
    sensorValueSum = 0;
    sensorValueCount = 0;
    sensorLastSampleMillis = time;
    
    bool sensorState = sensorValue >= sensorReadingPositiveThreshold;

    if (sensorState != prevSensorState) {
      prevSensorState = sensorState;
      lightSensorEventQueue.addNewEvent(ValueChangeEvent(sensorState ? 1 : 0));

      if (sensorState == false) {
        startSiren();
      }
    }
  }

  bool digitalValue = !digitalRead(movementSensor0Pin);
  
  if (digitalValue != movementSensor0Value) {
    movementSensor0Value = digitalValue;
    movementSensor0EventQueue.addNewEvent(ValueChangeEvent(movementSensor0Value ? 1 : 0));

    if (movementSensor0Value == true) {
      startSiren();
    }
  }

  if(sirenStartedMillis > -1 && time - sirenStartedMillis >= sirenTimeoutMillis) {
    stopSiren();
  }

  if (clientConnected) {
    ValueChangeEvent* event = lightSensorEventQueue.getNextEvent();

    if (event != NULL) {
      client.print(sectorText);
      client.println(event->value);
    }

    event = movementSensor0EventQueue.getNextEvent();
    
    if (event != NULL) {
      client.print(movement0Text);
      client.println(event->value);
    }

    int idx = serverCommand.lastIndexOf(NEW_LINE);
    if(idx >= 0)
    {
      serverCommand.remove(idx); // remove newline at the end
      
      if (serverCommand == statusQueryText) {
        client.print(sectorText);
        client.print(prevSensorState ? 1 : 0);
        client.print(',');
        client.print(movement0Text);
        client.print(movementSensor0Value ? 1 : 0);
        client.print(',');
        client.print(rssiText);
        client.println(WiFi.RSSI());
      } else if(serverCommand == stopSirenText) {
        stopSiren();
      } else {
        idx = serverCommand.indexOf(':');
        if (idx >= 0 && idx < serverCommand.length() - 1) {
          String parameterName = serverCommand.substring(0, idx);
          String parameterValue = serverCommand.substring(idx + 1);
          int integerValue = parameterValue.toInt();
          
          if (parameterName == "sirenActivated") {
            sirenActivated = integerValue > 0;
  
            if (!sirenActivated)
              stopSiren();
          } else if (parameterName == "readingSamplingMillis") {
            readingSamplingMillis = integerValue;
          } else if (parameterName == "sensorReadingPositiveThreshold") {
            sensorReadingPositiveThreshold = integerValue;
          } else if (parameterName == "sirenTimeoutMillis") {
            sirenTimeoutMillis = integerValue;
          }
        }
      }

      serverCommand.remove(0);
    }
  }
}
