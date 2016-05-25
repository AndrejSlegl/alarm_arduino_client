#include <SPI.h>
#include <WiFi101.h>
#include "WiFiPersistentConnector.h"

//#define DEBUG

class ValueChangeEvent {
  public:
  bool isSet = false;
  byte value = 0;

#ifdef DEBUG
  String text;
  ValueChangeEvent(String text) {
    this->text = text;
  }
#endif

  ValueChangeEvent() {
  }
  
  ValueChangeEvent(byte value) {
    this->value = value;
  }
};

const char NEW_LINE[] = "\r\n";
const int lightSensorPin = 0;
const int clientLedPin = 6;
const int sensorReadingPositiveThreshold = 980;
const int readingSamplingMillis = 50;

const char sectorText[] = "sector0:";
const char rssiText[] = "rssi:";
const char statusQueryText[] = "status?";
char server[] = "192.168.1.111";
int port = 43254;
WiFiPersistentConnector connector("SLEGL WiFi", "pnr41wlan");
WiFiClient client;
bool wifiConnected = false;
bool clientConnected = false;
bool clientConnectionOn = true;

const int eventQueueSize = 20;
ValueChangeEvent eventQueue[eventQueueSize];
ValueChangeEvent *currentEvent;
ValueChangeEvent *eventQueueFirst;
ValueChangeEvent *eventQueueLast;
bool prevSensorState = false;
int sensorValueSum = 0;
int sensorValueCount = 0;
int sensorLastSampleMillis = 0;

void moveToNext(ValueChangeEvent **ppEvent) {
  if (*ppEvent == eventQueueLast)
    *ppEvent = eventQueueFirst;
  else
    *ppEvent = *ppEvent + 1;
}

void addNewEvent(const ValueChangeEvent &newEvent) {
  ValueChangeEvent *event = currentEvent;

  while (event->isSet) {
    moveToNext(&event);
      
    if (event == currentEvent)
      break;
  }

  if (event->isSet) {
    moveToNext(&currentEvent);
  }

  *event = newEvent;
  event->isSet = true;
}

ValueChangeEvent* getNextEvent() {
  ValueChangeEvent *event = currentEvent;

  if(!event->isSet)
    return NULL;

  event->isSet = false;
  moveToNext(&currentEvent);

  return event;
}

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

#ifdef DEBUG
void printEventQueue() {
  int i=0;
  while(i < eventQueueSize) {
    ValueChangeEvent *event = &eventQueue[i];
    
    Serial.print(i);
    Serial.print(": ");
    if (event->isSet)
      Serial.print(event->text);
    else
      Serial.print("EMPTY");

    if (event == currentEvent)
      Serial.print(" --");
    Serial.println();
    ++i;
  }

  Serial.println();
}
#endif

void setup() {
  eventQueueFirst = eventQueue;
  eventQueueLast = eventQueue + (eventQueueSize - 1);
  currentEvent = eventQueueFirst;

#ifdef DEBUG
  Serial.begin(9600);
  //while (!Serial);
#else
  clientConnectionOn = true;
#endif

  pinMode(clientLedPin, OUTPUT);
  connector.start();
}

void loop() {
  connector.update();

  bool wifiCon = WiFi.status() == WL_CONNECTED;
  if(wifiCon != wifiConnected) {
    wifiConnected = wifiCon;

#ifdef DEBUG
    Serial.println(wifiConnected ? "WiFi Connected" : "WiFi Disconnected");
#endif
  }

  String serverCommand;
  while (client.available() && serverCommand.lastIndexOf(NEW_LINE) < 0) {
    char c = client.read();
    serverCommand += c;
    
#ifdef DEBUG
    //Serial.print(c);
#endif
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
    else if (command.startsWith("put")) {
      addNewEvent(ValueChangeEvent(command));
      printEventQueue();
    }
    else if (command.startsWith("get")) {
      const ValueChangeEvent *event = getNextEvent();
      if (event == NULL)
        Serial.println("NULL");
      else {
        Serial.println(event->text);
      }
      Serial.println();
    }
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
      addNewEvent(ValueChangeEvent(sensorState ? 1 : 0));
  
#ifdef DEBUG
      int count = 0;
      for (int i=0; i<eventQueueSize; ++i){
        if (eventQueue[i].isSet)
          ++count;
      }
  
      Serial.print("Event count = ");
      Serial.println(count);
#endif
    }
  }

  if (clientConnected) {
    const ValueChangeEvent* event = getNextEvent();

    if (event != NULL) {
      client.print(sectorText);
      client.println(event->value);
    }

    if (serverCommand.startsWith(statusQueryText)) {
      client.print(sectorText);
      client.print(prevSensorState ? 1 : 0);
      client.print(',');
      client.print(rssiText);
      client.println(WiFi.RSSI());
    }

    /*
    long rssi = WiFi.RSSI();
    client.print("rssi:");
    client.println(rssi);
    client.print("sensorValue:");
    client.println(sensorValue);
    delay(100);
    */
  }

#ifdef DEBUG
  if (clientConnected) {
    if (command == "send") {
      Serial.println("Sending request");
      client.print(sectorText);
      client.println(prevSensorState ? 1 : 0);
    }
  }
#endif

}
