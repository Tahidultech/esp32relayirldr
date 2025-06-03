#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <IRremote.h>
#include <EEPROM.h>

// Device Info
const char *deviceName = "ESP32 Controller";
const char *roomName = "Living Room";

// GPIO Pins
#define RELAY1_PIN 16
#define RELAY2_PIN 17  
#define RELAY3_PIN 18  // Light
#define RELAY4_PIN 19  // Fan
#define FAN_PWM_PIN 23
#define IR_PIN 35
#define LDR_PIN 34

// Buttons
#define BTN_RELAY1 25
#define BTN_RELAY2 26
#define BTN_RELAY3 27
#define BTN_RELAY4 14
#define BTN_FAN_UP 12
#define BTN_FAN_DOWN 13

// PWM Fan settings
#define FAN_PWM_FREQ 5000
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_RESOLUTION 8
int fanSpeed = 0;  // 0-4 (0=OFF, 4=MAX)

// LDR thresholds
#define DARK_THRESHOLD 500
bool isRoomDark = false;
int signalLossCounter = 0;
int lastLdrValue = 0;

// EEPROM settings
#define EEPROM_SIZE 10
#define RELAY1_ADDR 0
#define RELAY2_ADDR 1
#define RELAY3_ADDR 2
#define RELAY4_ADDR 3
#define FAN_SPEED_ADDR 4

// RainMaker devices
static Switch sw1("Switch1", &RELAY1_PIN);
static Switch sw2("Switch2", &RELAY2_PIN);
static Switch light("Light", &RELAY3_PIN);
static Fan fan("Fan", &RELAY4_PIN);
static Device signalStatus("Status");

// IR Receiver
IRrecv irrecv(IR_PIN);
decode_results irResults;

// Button states
bool btn1State = HIGH;
bool btn2State = HIGH;
bool btn3State = HIGH;
bool btn4State = HIGH;
bool btnFanUpState = HIGH;
bool btnFanDownState = HIGH;

// Function prototypes
void setupRainMaker();
void setupGPIO();
void loadStateFromEEPROM();
void saveState(int addr, bool state);
void saveFanSpeed();
void sysProvEvent(arduino_event_t *sys_event);
void handleButtons();
void updateLDR();
void writeDevicesToEEPROM();
void fanSpeedControl(int change);

// Callback functions for RainMaker
void writeSw1Callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx);
void writeSw2Callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx);
void writeLightCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx);
void writeFanCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx);

void setup() {
  Serial.begin(115200);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Setup GPIO
  setupGPIO();
  
  // Load last states from EEPROM
  loadStateFromEEPROM();
  
  // Start IR receiver in decode mode
  irrecv.enableIRIn();
  Serial.println("IR Receiver started in decode mode. Send IR signals to see HEX codes.");
  
  // Setup RainMaker
  setupRainMaker();
}

void loop() {
  // Handle IR commands
  if (irrecv.decode(&irResults)) {
    Serial.print("IR Code: 0x");
    Serial.println(irResults.value, HEX);
    
    // Handle IR commands here
    handleIRCommands(irResults.value);
    
    irrecv.resume();
  }
  
  // Handle manual buttons
  handleButtons();
  
  // Check LDR sensor
  updateLDR();
  
  // Small delay
  delay(50);
}

void setupRainMaker() {
  Node my_node;
  my_node = RMaker.initNode(deviceName);
  
  // Add switches with custom names
  sw1.addCb(writeSw1Callback);
  light.addCb(writeLightCallback);
  sw2.addCb(writeSw2Callback);
  
  // Add fan with speed control
  fan.addCb(writeFanCallback);
  
  // Add parameter to fan for speed control (0-4)
  Param fanSpeedParam("speed", "custom.param.speed", value(fanSpeed), PARAM_RANGE(0, 4, 1));
  fan.addParam(fanSpeedParam);
  fan.updateAndReportParam("speed", fanSpeed);
  
  // Add status device with parameters
  Param roomLightParam("room_light", "custom.param.light", value("Unknown"), PARAM_TEXT);
  Param signalLossParam("signal_loss", "custom.param.signalloss", value(0), PARAM_NUMBER);
  
  signalStatus.addParam(roomLightParam);
  signalStatus.addParam(signalLossParam);
  
  // Add all devices to node
  my_node.addDevice(sw1);
  my_node.addDevice(sw2);
  my_node.addDevice(light);
  my_node.addDevice(fan);
  my_node.addDevice(signalStatus);
  
  // Start WiFi provisioning with BLE
  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  WiFi.onEvent(sysProvEvent);
  
  RMaker.start();
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, 
                         WIFI_PROV_SECURITY_1, "RainMakerPassphrase", roomName);
}

void setupGPIO() {
  // Setup output pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  
  // Setup input pins
  pinMode(BTN_RELAY1, INPUT_PULLUP);
  pinMode(BTN_RELAY2, INPUT_PULLUP);
  pinMode(BTN_RELAY3, INPUT_PULLUP);
  pinMode(BTN_RELAY4, INPUT_PULLUP);
  pinMode(BTN_FAN_UP, INPUT_PULLUP);
  pinMode(BTN_FAN_DOWN, INPUT_PULLUP);
  pinMode(LDR_PIN, INPUT);
  
  // Setup PWM for fan control
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
}

void loadStateFromEEPROM() {
  bool sw1State = EEPROM.read(RELAY1_ADDR);
  bool sw2State = EEPROM.read(RELAY2_ADDR);
  bool lightState = EEPROM.read(RELAY3_ADDR);
  bool fanState = EEPROM.read(RELAY4_ADDR);
  fanSpeed = EEPROM.read(FAN_SPEED_ADDR);
  
  // Continuing from line 196:
  if (fanSpeed > 4) fanSpeed = 0; // Validate fan speed
  
  digitalWrite(RELAY1_PIN, sw1State);
  digitalWrite(RELAY2_PIN, sw2State);
  digitalWrite(RELAY3_PIN, lightState);
  digitalWrite(RELAY4_PIN, fanState);
  
  // Set initial PWM value for fan
  if (fanState && fanSpeed > 0) {
    int pwmValue = map(fanSpeed, 1, 4, 64, 255);
    ledcWrite(FAN_PWM_CHANNEL, pwmValue);
  } else {
    ledcWrite(FAN_PWM_CHANNEL, 0);
  }
  
  // Update RainMaker device states
  sw1.updateAndReportParam("power", sw1State);
  sw2.updateAndReportParam("power", sw2State);
  light.updateAndReportParam("power", lightState);
  fan.updateAndReportParam("power", fanState);
  fan.updateAndReportParam("speed", fanSpeed);
}

void handleIRCommands(unsigned long irCode) {
  switch(irCode) {
    case 0xFF6897: // Example: Button 1 for Switch 1
      digitalWrite(RELAY1_PIN, !digitalRead(RELAY1_PIN));
      sw1.updateAndReportParam("power", digitalRead(RELAY1_PIN));
      saveState(RELAY1_ADDR, digitalRead(RELAY1_PIN));
      break;
    case 0xFF9867: // Example: Button 2 for Switch 2
      digitalWrite(RELAY2_PIN, !digitalRead(RELAY2_PIN));
      sw2.updateAndReportParam("power", digitalRead(RELAY2_PIN));
      saveState(RELAY2_ADDR, digitalRead(RELAY2_PIN));
      break;
    case 0xFFB04F: // Example: Button 3 for Light
      digitalWrite(RELAY3_PIN, !digitalRead(RELAY3_PIN));
      light.updateAndReportParam("power", digitalRead(RELAY3_PIN));
      saveState(RELAY3_ADDR, digitalRead(RELAY3_PIN));
      break;
    case 0xFF30CF: // Example: Button 4 for Fan toggle
      digitalWrite(RELAY4_PIN, !digitalRead(RELAY4_PIN));
      fan.updateAndReportParam("power", digitalRead(RELAY4_PIN));
      saveState(RELAY4_ADDR, digitalRead(RELAY4_PIN));
      
      // Handle fan PWM when toggling
      if (digitalRead(RELAY4_PIN) && fanSpeed > 0) {
        int pwmValue = map(fanSpeed, 1, 4, 64, 255);
        ledcWrite(FAN_PWM_CHANNEL, pwmValue);
      } else {
        ledcWrite(FAN_PWM_CHANNEL, 0);
      }
      break;
    case 0xFF18E7: // Example: Fan speed up
      fanSpeedControl(1);
      break;
    case 0xFF4AB5: // Example: Fan speed down
      fanSpeedControl(-1);
      break;
  }
}

void handleButtons() {
  // Check Switch 1 button
  bool currentBtn1 = digitalRead(BTN_RELAY1);
  if (currentBtn1 == LOW && btn1State == HIGH) {
    digitalWrite(RELAY1_PIN, !digitalRead(RELAY1_PIN));
    sw1.updateAndReportParam("power", digitalRead(RELAY1_PIN));
    saveState(RELAY1_ADDR, digitalRead(RELAY1_PIN));
    delay(50); // Debounce
  }
  btn1State = currentBtn1;
  
  // Check Switch 2 button
  bool currentBtn2 = digitalRead(BTN_RELAY2);
  if (currentBtn2 == LOW && btn2State == HIGH) {
    digitalWrite(RELAY2_PIN, !digitalRead(RELAY2_PIN));
    sw2.updateAndReportParam("power", digitalRead(RELAY2_PIN));
    saveState(RELAY2_ADDR, digitalRead(RELAY2_PIN));
    delay(50); // Debounce
  }
  btn2State = currentBtn2;
  
  // Check Light button
  bool currentBtn3 = digitalRead(BTN_RELAY3);
  if (currentBtn3 == LOW && btn3State == HIGH) {
    digitalWrite(RELAY3_PIN, !digitalRead(RELAY3_PIN));
    light.updateAndReportParam("power", digitalRead(RELAY3_PIN));
    saveState(RELAY3_ADDR, digitalRead(RELAY3_PIN));
    delay(50); // Debounce
  }
  btn3State = currentBtn3;
  
  // Check Fan button
  bool currentBtn4 = digitalRead(BTN_RELAY4);
  if (currentBtn4 == LOW && btn4State == HIGH) {
    digitalWrite(RELAY4_PIN, !digitalRead(RELAY4_PIN));
    fan.updateAndReportParam("power", digitalRead(RELAY4_PIN));
    saveState(RELAY4_ADDR, digitalRead(RELAY4_PIN));
    
    // Handle fan PWM when toggling
    if (digitalRead(RELAY4_PIN) && fanSpeed > 0) {
      int pwmValue = map(fanSpeed, 1, 4, 64, 255);
      ledcWrite(FAN_PWM_CHANNEL, pwmValue);
    } else {
      ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    delay(50); // Debounce
  }
  btn4State = currentBtn4;
  
  // Check Fan Up button
  bool currentBtnFanUp = digitalRead(BTN_FAN_UP);
  if (currentBtnFanUp == LOW && btnFanUpState == HIGH) {
    fanSpeedControl(1);
    delay(50); // Debounce
  }
  btnFanUpState = currentBtnFanUp;
  
  // Check Fan Down button
  bool currentBtnFanDown = digitalRead(BTN_FAN_DOWN);
  if (currentBtnFanDown == LOW && btnFanDownState == HIGH) {
    fanSpeedControl(-1);
    delay(50); // Debounce
  }
  btnFanDownState = currentBtnFanDown;
}

void fanSpeedControl(int change) {
  // Only change speed if fan is on
  if (digitalRead(RELAY4_PIN)) {
    fanSpeed += change;
    
    // Ensure fan speed stays within limits
    if (fanSpeed < 0) fanSpeed = 0;
    if (fanSpeed > 4) fanSpeed = 4;
    
    // Update PWM output
    if (fanSpeed > 0) {
      int pwmValue = map(fanSpeed, 1, 4, 64, 255);
      ledcWrite(FAN_PWM_CHANNEL, pwmValue);
    } else {
      ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    
    // Update RainMaker and save state
    fan.updateAndReportParam("speed", fanSpeed);
    saveFanSpeed();
  }
}

void updateLDR() {
  int ldrValue = analogRead(LDR_PIN);
  
  // Update room light status
  bool previousDark = isRoomDark;
  isRoomDark = ldrValue < DARK_THRESHOLD;
  
  // Check for signal loss
  if (abs(ldrValue - lastLdrValue) > 1000) {
    signalLossCounter++;
    signalStatus.updateAndReportParam("signal_loss", signalLossCounter);
  }
  lastLdrValue = ldrValue;
  
  // Update room light parameter
  if (isRoomDark != previousDark) {
    const char* lightStatus = isRoomDark ? "Dark" : "Bright";
signalStatus.updateAndReportParam("room_light", lightStatus);
  }
}

void saveState(int addr, bool state) {
  EEPROM.write(addr, state);
  EEPROM.commit();
}

void saveFanSpeed() {
  EEPROM.write(FAN_SPEED_ADDR, fanSpeed);
  EEPROM.commit();
}

void writeDevicesToEEPROM() {
  EEPROM.write(RELAY1_ADDR, digitalRead(RELAY1_PIN));
  EEPROM.write(RELAY2_ADDR, digitalRead(RELAY2_PIN));
  EEPROM.write(RELAY3_ADDR, digitalRead(RELAY3_PIN));
  EEPROM.write(RELAY4_ADDR, digitalRead(RELAY4_PIN));
  EEPROM.write(FAN_SPEED_ADDR, fanSpeed);
  EEPROM.commit();
}

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Connected to WiFi. IP: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      break;
    default:
      break;
  }
}

// RainMaker callback functions
void writeSw1Callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  bool state = val.val.b;
  digitalWrite(RELAY1_PIN, state);
  saveState(RELAY1_ADDR, state);
  param->updateAndReport(val);
}

void writeSw2Callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  bool state = val.val.b;
  digitalWrite(RELAY2_PIN, state);
  saveState(RELAY2_ADDR, state);
  param->updateAndReport(val);
}

void writeLightCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  bool state = val.val.b;
  digitalWrite(RELAY3_PIN, state);
  saveState(RELAY3_ADDR, state);
  param->updateAndReport(val);
}

void writeFanCallback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  if (strcmp(param->getName(), "power") == 0) {
    bool state = val.val.b;
    digitalWrite(RELAY4_PIN, state);
    saveState(RELAY4_ADDR, state);
    
    // Handle fan PWM when toggling
    if (state && fanSpeed > 0) {
      int pwmValue = map(fanSpeed, 1, 4, 64, 255);
      ledcWrite(FAN_PWM_CHANNEL, pwmValue);
    } else {
      ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    
    param->updateAndReport(val);
  } else if (strcmp(param->getName(), "speed") == 0) {
    fanSpeed = val.val.i;
    
    // Validate fan speed
    if (fanSpeed < 0) fanSpeed = 0;
    if (fanSpeed > 4) fanSpeed = 4;
    
    // Update PWM if fan is currently on
    if (digitalRead(RELAY4_PIN) && fanSpeed > 0) {
      int pwmValue = map(fanSpeed, 1, 4, 64, 255);
      ledcWrite(FAN_PWM_CHANNEL, pwmValue);
    } else if (fanSpeed == 0) {
      ledcWrite(FAN_PWM_CHANNEL, 0);
    }
    
    saveFanSpeed();
    param->updateAndReport(val);
  }
}