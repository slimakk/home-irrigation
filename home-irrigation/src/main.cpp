#include <Arduino.h>
#include <AsyncMqtt_Generic.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

#define VOLTAGE_PIN A0
#define A_PIN D4
#define B_PIN D3
#define C_PIN D2

#define MQTT_HOST IPAddress(192, 168, 100, 24)
#define MQTT_PORT 1883

#define PANEL_U_TOPIC "zavlaha/voltage/panel"
#define BATTERY_U_TOPIC "zavlaha/voltage/battery"
#define PANEL_I_TOPIC "zavlaha/current/panel"
#define BATTERY_I_TOPIC "zavlaha/current/battery"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

Ticker wifiReconnectTimer;

WiFiManager wifiManager;

float resolution = 3.3 / 1023;
float sensitivity = 0.185;
/*
  ADC Calibration
  1. Max desired input voltage - 24V
  2. Used resitor values - 1M, 1M, 100k + 220k and 100k on board
  3. Full scale voltage value from divider within ADC domain 0 - 1 V
      Vout = (R2/(R1+R2))*Vin = 0.992V
  4. Voltage divider ratio = Max Input Voltage / Full scale voltage
  5. Read ADC value - analogRead(VOlTAGE_PIN) < 1024
  6. Calculate LSB
      LSB = Input voltage measured by multi / ADC value
      LSB = (ADC Input voltage on ESP / ADC value) * V divider ratio
  7. Real voltage = ADC value * LSB
*/
float LSB = 0.02314363968;

unsigned long previousMillis = 0;
const long interval = 3000;

void connectToWifi(){
  Serial.println("Connecting to wifi...");
  //WiFi.begin();
  wifiManager.autoConnect();
}

void connectToMQTT(){
  Serial.println("Connecting to MQTT broker...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event){
  (void) event;
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());
  connectToMQTT();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event){
  (void) event;
  Serial.println("Disconnected");
  mqttReconnectTimer.detach();
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMQTTConnect(bool sessionPresent){
  Serial.print("Connected to MQTT Broker: ");
  Serial.println(MQTT_HOST);
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetPanelSub1 = mqttClient.subscribe(PANEL_U_TOPIC, 0);
  Serial.println(packetPanelSub1);
  uint16_t packetPanelSub2 = mqttClient.subscribe(PANEL_I_TOPIC, 0);
  Serial.println(packetPanelSub2);
  uint16_t packetBatterySub1 = mqttClient.subscribe(BATTERY_U_TOPIC, 0);
  Serial.println(packetBatterySub1);
  uint16_t packetBatterySub2 = mqttClient.subscribe(BATTERY_I_TOPIC, 0);
  Serial.println(packetBatterySub2);
}

void onMQTTDisconnect(AsyncMqttClientDisconnectReason reason){
  (void) reason;
  Serial.println("Disconnected from MQTT broker");
  if(WiFi.isConnected()){
    mqttReconnectTimer.once(2, connectToMQTT);
  }
}

void onMQTTSubscribe(const uint16_t& packetID, const uint8_t qos){
  Serial.println("Subscribe acknowledged");
  Serial.print("PacketID: "); Serial.println(packetID);
  Serial.print("QoS"); Serial.println(qos);
}

void onMQTTUnsubscribe(const uint16_t& packetID){
  Serial.println("Unsubscribe acknowledged");
  Serial.print("PacketID: "); Serial.println(packetID);
}

void onMQTTPublish(const uint16_t& packetID){
  Serial.println("Publish acknowledged");
  Serial.print("PacketID: "); Serial.println(packetID);
}

void onMQTTMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties, const size_t& len, const size_t& index, const size_t& total){
  (void) payload;
  Serial.println("Publish received");
  Serial.print("  topic: ");  Serial.println(topic);
}

void changeInput(int c, int b, int a){
  digitalWrite(A_PIN, a);
  digitalWrite(B_PIN, b);
  digitalWrite(C_PIN, c);
}

void measureCurrent(int sensor) {
  float current = 0, voltage = 0;
  int adc;
  if(sensor == 0){
    changeInput(LOW, LOW, LOW);
    Serial.println("Panel current");
    for(int i = 1; i<= 10; i++){
      Serial.print(" . ");
    }
    Serial.println("");
    adc = analogRead(VOLTAGE_PIN);
    voltage = adc*resolution;
    current = (voltage-2.5) / sensitivity;
    if(current < 0.05){
      current = 0;
    }
    mqttClient.publish(PANEL_I_TOPIC, 2, true, String(current).c_str());
  }
  else if(sensor == 1){
    changeInput(LOW, LOW, HIGH);
    Serial.println("Battery current");
    for(int i = 1; i<= 10; i++){
      Serial.print(" . ");
    }
    Serial.println("");
    adc = analogRead(VOLTAGE_PIN);
    voltage = adc*resolution;
    current = (voltage-2.5) / sensitivity;
    if(current < 0.05){
      current = 0;
    }
    mqttClient.publish(BATTERY_I_TOPIC, 2, true, String(current).c_str());
  }
  Serial.print("Current = ");
  Serial.println(current);
}

void measureVoltage(int sensor){
  int adc; float voltage;
  if(sensor == 0){
    changeInput(HIGH, LOW, LOW);
    Serial.println("Panel voltage");
    adc = analogRead(VOLTAGE_PIN);
    voltage = adc*LSB;
    /*Serial.print("ADC = ");
    Serial.println(adc);*/
    mqttClient.publish(PANEL_U_TOPIC, 2, true, String(voltage).c_str());
    Serial.print("Voltage = ");
    Serial.println(voltage, 3);
  }
  else if(sensor == 1){
    changeInput(HIGH, LOW, HIGH);
    Serial.println("Battery voltage");
    adc = analogRead(VOLTAGE_PIN);
    voltage = adc*LSB;
    /*Serial.print("ADC = ");
    Serial.println(adc);*/
    mqttClient.publish(BATTERY_U_TOPIC, 2, true, String(voltage).c_str());
    Serial.print("Voltage = ");
    Serial.println(voltage, 3);
  }
  
}

void setup() {
  Serial.begin(115200);
  
  wifiManager.autoConnect();
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(C_PIN, OUTPUT);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMQTTConnect);
  mqttClient.onDisconnect(onMQTTDisconnect);
  mqttClient.onSubscribe(onMQTTSubscribe);
  mqttClient.onUnsubscribe(onMQTTUnsubscribe);
  mqttClient.onMessage(onMQTTMessage);
  mqttClient.onPublish(onMQTTPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  //connectToWifi();
  connectToMQTT();
}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    measureCurrent(0);
    measureVoltage(0);
  }
}