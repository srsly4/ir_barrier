#define DECODE_ONKYO
#include <IRremote.hpp>

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoHA.h>

#define STATE_UNPAIRED 0
#define STATE_PAIRED 1
#define STATE_LOW_BATTERY 2

#define INTERVAL_MS 600
#define PAIRED_MESSAGES_COUNT 20
#define OBSTACLE_DETECT_MS 1000
#define LOW_BATTERY_THRESHOLD 290
#define HIGH_BATTERY_THRESHOLD 420

#define WIFI_SSID "SiriusLAN_Legacy"
#define WIFI_PASSWORD ""

#define MQTT_SERVER "10.0.44.2"
#define MQTT_SERVER_PORT 1883

#define VOLTAGE_MEASUREMENTS 50
#define VOLTAGE_REPORT_INTERVAL_MS (5 * 60 * 1000)

#define UNPAIRED_BLINK_INTERVAL 500
#define MAX_MOVEMENT_TIME 20*1000

byte state = STATE_UNPAIRED;

long int last_received = 0;
int unpaired_consecutive_messages = 0;

const int irReceiverPin = 4;

// wifi checks
unsigned long previousMillis = 0;
unsigned long interval = 30000;
unsigned long lastDetectTime = 0;

// unpaired led blink
unsigned long lastBlinkSwitch = 0;
int blink = 0;


int voltages[VOLTAGE_MEASUREMENTS];
int voltage_ndx = 0;
unsigned long voltage_last_report_ms = 0;

WiFiClient client;
byte mac[] = {0x55, 0x55, 0x50, 0x00, 0x00, 0x01};
HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device);
HABinarySensor sensor("barrier");
HASensorNumber batSensor("transmitter_battery", HASensorNumber::PrecisionP1);

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  mqtt.begin(MQTT_SERVER, MQTT_SERVER_PORT);
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void mqtt_onConnected() {
  Serial.println("MQTT - connected");
}

void mqtt_onDisconnected() {
  Serial.println("MQTT - disconnected!");
}

void setup() {
  for (int i = 0; i < VOLTAGE_MEASUREMENTS; i++) {
    voltages[i] = 0;
  }
  Serial.begin(115200);

  device.setName("IR Barrier Sensor");
  device.setSoftwareVersion("0.1.1");
  device.setManufacturer("Szymon Piechaczek");
  device.setModel("IR-01");
  device.enableLastWill();

  sensor.setCurrentState(false);
  sensor.setName("IR Barrier");
  sensor.setDeviceClass("motion");
  sensor.setIcon("mdi:motion-sensor");

  batSensor.setName("Transmitter battery");
  batSensor.setDeviceClass("battery");
  batSensor.setIcon("mdi:battery");
  batSensor.setUnitOfMeasurement("%");

  mqtt.onConnected(mqtt_onConnected);
  mqtt.onDisconnected(mqtt_onDisconnected);
  
  IrReceiver.begin(irReceiverPin, false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  pinMode(LED_BUILTIN, OUTPUT);
  blink = 0;
  digitalWrite(LED_BUILTIN, blink);
  
  Serial.println("Setup done");
}

void process_payload(uint16_t bat_level) {
  long int curr = millis();
  long int since = curr - last_received;
  last_received = curr;

  Serial.printf("Since last payload: %d\r\n", since);
  
  switch (state) {
    case STATE_UNPAIRED:
      if (since < INTERVAL_MS) {
        unpaired_consecutive_messages += 1;
        if (unpaired_consecutive_messages >= PAIRED_MESSAGES_COUNT) {
          state = STATE_PAIRED;
          Serial.printf("Paired!\r\n");
          unpaired_consecutive_messages = 0;
          sensor.setState(false);
          blink = 0;
          digitalWrite(LED_BUILTIN, blink);
        }
      } else {
        unpaired_consecutive_messages = 0;
      } 
    break;
    case STATE_PAIRED:
    break;
    
    default:
    break;
    }

  // calculate mean voltage
  voltages[voltage_ndx] = bat_level;
  voltage_ndx = (voltage_ndx + 1) % VOLTAGE_MEASUREMENTS;
  if (millis() - voltage_last_report_ms > VOLTAGE_REPORT_INTERVAL_MS && voltage_ndx == (VOLTAGE_MEASUREMENTS - 1)) {
    voltage_last_report_ms = millis();
    int total = 0;
    for (int i = 0; i < VOLTAGE_MEASUREMENTS; i++) {
      total += voltages[i];
    }
    int average = total / VOLTAGE_MEASUREMENTS;
    float voltage = ((float)average) / 100.0f;
    float normalized = ((float)(average - LOW_BATTERY_THRESHOLD) / (float)(HIGH_BATTERY_THRESHOLD - LOW_BATTERY_THRESHOLD));
    Serial.printf("VOLTAGE percent: %f\r\n", normalized);

    batSensor.setValue(
      100.0f * normalized
    );
    if (average < LOW_BATTERY_THRESHOLD) {
      device.setAvailability(false);
      state = STATE_LOW_BATTERY;
    } else {
      device.setAvailability(true);
      if (state == STATE_LOW_BATTERY) {
        state = STATE_UNPAIRED;
      }
    }
  } 
  
}

void timer_check() {
  long int curr = millis();
  long int since = curr - last_received;
  if (state == STATE_PAIRED && since >= OBSTACLE_DETECT_MS) {
        Serial.printf("EVENT: Obstacle detected!\r\n");
        lastDetectTime = curr;
        sensor.setState(true, true);
        state = STATE_UNPAIRED;
        unpaired_consecutive_messages = 0;
  }
  if (sensor.getCurrentState() && curr - lastDetectTime > MAX_MOVEMENT_TIME) {
    sensor.setState(false);
  }
}

void loop() {
  mqtt.loop();
  // IR check
  if (IrReceiver.decode()) {
    IRData* message = IrReceiver.read();
    //IrReceiver.printIRResultShort(&Serial);
    if (message->protocol == ONKYO && message->address == 0x1221) {
      Serial.printf("Received 0x%02X, power: %d\r\n", message->command, message->command);
      process_payload(message->command);
    }
    IrReceiver.resume();
  }
  timer_check();

  // blink if unpaired
  if (state == STATE_UNPAIRED && millis() - lastBlinkSwitch > UNPAIRED_BLINK_INTERVAL) {
    lastBlinkSwitch = millis();
    blink = 1 - blink;
    digitalWrite(LED_BUILTIN, blink);
  }
  
}
