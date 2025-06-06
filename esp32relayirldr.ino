#include <WiFi.h>
#include "RMaker.h"
#include "WiFiProv.h"

#define RELAY1_PIN 4
#define RELAY2_PIN 5
#define RELAY3_PIN 18
#define RELAY4_PIN 19

// Relay devices
static Switch my_switch1("Light 1", &RELAY1_PIN);
static Switch my_switch2("Light 2", &RELAY2_PIN);
static Switch my_switch3("Light 3", &RELAY3_PIN);
static Switch my_switch4("Fan",     &RELAY4_PIN);

// Node name
const char *service_name = "PROV_4RELAY";
const char *pop = "12345678";

void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("Provisioning Started. Name: %s, POP: %s\n", service_name, pop);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Wi-Fi Connected!");
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("Wi-Fi Credentials Received.");
      break;
    case ARDUINO_EVENT_PROV_SUCCESS:
      Serial.println("Provisioning Successful.");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("Provisioning Ended.");
      break;
    default:
      break;
  }
}

void setupRelays() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  // Default all relays OFF
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  setupRelays();

  // RainMaker Node
  Node my_node;
  my_node = RMaker.initNode("ESP32 4-Relay");

  // Add Switches to Node
  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_node.addDevice(my_switch4);

  // Enable OTA, Schedule, TimeSync (optional)
  RMaker.enableOTA();
  RMaker.enableSchedule();
  RMaker.enableTZService();
  RMaker.enableSchedule();

  RMaker.start();

  // WiFi provisioning using BLE
  WiFi.onEvent(sysProvEvent);
  WiFiProv.begin(ESP_MAC, BLE, service_name, pop);
}

void loop()
{
  // Nothing here unless needed for other features
}
