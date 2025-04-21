#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT20.h>
#include <TaskScheduler.h>
#include <LiquidCrystal_I2C.h>

// WiFi & Ethernet Configuration
// #define WIFI_SSID "ACLAB"
// #define WIFI_PASSWORD "ACLAB2023"
#define WIFI_SSID "anhtuan"
#define WIFI_PASSWORD "anhtuan24"
WiFiClient wifiClient;

// Adafruit IO Credentials
#define AIO_USERNAME "tranhaithaoquang"
#define AIO_KEY "aio_LVBx634Jpqp25FtzNmVtl7zeCstl"
#define FEED_TEMP "temperature"
#define FEED_HUMIDITY "humidity"
#define FEED_MOISTURE "soil-moisture"
#define FEED_PUMP_STATE "pump-state"
#define FEED_PUMP_MODE "pump-mode"
#define FEED_PUMP "relay-to-pump"
#define FEED_L_TH "moisture-threshold-low"
#define FEED_H_TH "moisture-threshold-high"

// MQTT Configuration
Adafruit_MQTT_Client mqtt(&wifiClient, "io.adafruit.com", 1883, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish moistureFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_MOISTURE);
Adafruit_MQTT_Publish tempFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_TEMP);
Adafruit_MQTT_Publish humidityFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_HUMIDITY);
Adafruit_MQTT_Publish pumpStateFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_PUMP_STATE);
Adafruit_MQTT_Subscribe pumpControl(&mqtt, AIO_USERNAME "/feeds/" FEED_PUMP);
Adafruit_MQTT_Subscribe modeControl(&mqtt, AIO_USERNAME "/feeds/" FEED_PUMP_MODE);
Adafruit_MQTT_Subscribe lThControl(&mqtt, AIO_USERNAME "/feeds/" FEED_L_TH);
Adafruit_MQTT_Subscribe hThControl(&mqtt, AIO_USERNAME "/feeds/" FEED_H_TH);

// Sensor & LCD & Button Configuration
#define SOIL_SENSOR_PIN 32
#define RELAY_PIN 33
#define BUTTON_PIN 35 // pull-up button
DHT20 dht20;
LiquidCrystal_I2C lcd(0x21, 16, 2);

#define NORMAL_STATE 1
#define PRESSED_STATE 0
int keyReg0 = NORMAL_STATE;
int keyReg1 = NORMAL_STATE;
int keyReg2 = NORMAL_STATE;
int keyReg3 = NORMAL_STATE;
int timeOutKeyPress = 500;
int button_pressed = 0;
int button_long_pressed = 0;
int button_flag = 0;
int isButtonPressed() {
    if(button_flag == 1){
        button_flag = 0;
        return 1;
    }
    return 0;
}
int isButtonLongPressed() {
    if(button_long_pressed == 1){
        button_long_pressed = 0;
        return 1;
    }
    return 0;
}
void readDebouncedButton() {
    keyReg2 = keyReg1;
    keyReg1 = keyReg0;
    keyReg0 = digitalRead(BUTTON_PIN);
    if ((keyReg1 == keyReg0) && keyReg1 == keyReg2) {
        if (keyReg2 != keyReg3) {
            keyReg3 = keyReg2;
            if (keyReg3 == PRESSED_STATE) {
                timeOutKeyPress = 500;
                button_flag = 1;
            }
        }
        else {
            timeOutKeyPress--;
            if (timeOutKeyPress == 0) {
                timeOutKeyPress = 500;
                if (keyReg3 == PRESSED_STATE) {
                    button_flag = 1;
                }
            }
        }
    }
}

// System Variables
enum PumpState { AUTO_PUMP, MANUAL_PUMP, AUTO_PUMP_OFF, AUTO_PUMP_ON, MANUAL_PUMP_ON, MANUAL_PUMP_OFF };
enum LCDState { SEN_INFO, PUMP_INFO };
enum RCV_MODE { AUTO, MAN };
enum RCV_STATE { OFFPUMP, ONPUMP };
PumpState pumpState = AUTO_PUMP;
LCDState lcdState = SEN_INFO;
RCV_MODE rcv_cmd_mode = AUTO;
RCV_STATE rcv_cmd_state = OFFPUMP;
int soilMoisture = 0;
float temperature = 0;
float humidity = 0;
int stateOfPump = 0;
int L_TH = 410; // Default low threshold
int H_TH = 2048; // Default high threshold

// Function Prototypes
void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println(" WiFi connection successful!");
}
void connectMQTT() {
    int8_t ret;
    if (mqtt.connected()) {
        return;
    }
    Serial.print("Connecting MQTT... ");
    while ((ret = mqtt.connect()) != 0) {
        Serial.println(mqtt.connectErrorString(ret));
    }
    Serial.println("MQTT connection successful!");
}
void readSensors() {
    dht20.read();
    soilMoisture = analogRead(SOIL_SENSOR_PIN);
    temperature = dht20.getTemperature();
    humidity = dht20.getHumidity();
}
void updateToFeed() {
    if (!mqtt.connected()) connectMQTT();
    mqtt.processPackets(100);
    pumpStateFeed.publish(stateOfPump);

    moistureFeed.publish((soilMoisture * 100) / 4096);
    tempFeed.publish(temperature);
    humidityFeed.publish(humidity);
    // Serial.println("Updated Adafruit IO!");
}
void updateToLCD() {
    switch(lcdState) {
        case SEN_INFO: {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Temp: "); lcd.print(temperature);
            lcd.print("C");
            lcd.setCursor(0, 1);
            lcd.print("Moist: "); lcd.print(soilMoisture);
            if (isButtonPressed()) {
                lcdState = PUMP_INFO;
            }
            break;
        }
        case PUMP_INFO: {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("PUMP: "); 
            if (stateOfPump) lcd.print("ON");
            else lcd.print("OFF");
            lcd.setCursor(0, 1);
            lcd.print("Mode: "); 
            lcd.print(pumpState);
            if (isButtonPressed()) {
                lcdState = SEN_INFO;
            }
            break;
        }
        default: {
            break;
        }
    }
}
void controlPump() {
    if (stateOfPump) {
        digitalWrite(RELAY_PIN, HIGH);
    }
    else {
        digitalWrite(RELAY_PIN, LOW);
    }
}
void globalPump() {
    switch (pumpState) {
        case AUTO_PUMP: {
            if (rcv_cmd_mode == MAN) {
                pumpState = MANUAL_PUMP;
            }
            break;
        }
        case MANUAL_PUMP: {
            if (rcv_cmd_mode == AUTO) {
                pumpState = AUTO_PUMP;
            }
            break;
        }
        default: {
            break;
        }
    }
}
void automaticPump() {
        switch (pumpState) {
        case AUTO_PUMP: {
            pumpState = AUTO_PUMP_OFF;
            stateOfPump = 0;
            break;
        }
        case AUTO_PUMP_OFF: {
            controlPump();
            if (rcv_cmd_mode == MAN) {
                pumpState = MANUAL_PUMP;
                break;
            }
            if (soilMoisture <= L_TH) {
                pumpState = AUTO_PUMP_ON;
                stateOfPump = 1;
                break;
            }
            break;
        }
        case AUTO_PUMP_ON: {
            controlPump();
            if (rcv_cmd_mode == MAN) {
                pumpState = MANUAL_PUMP;
                break;
            }
            if (soilMoisture >= H_TH) {
                pumpState = AUTO_PUMP_OFF;
                stateOfPump = 0;
                break;
            }
            break;
        }
        default: {
            break;
        }
    }
}
void manualPump() {
    switch (pumpState) {
        case MANUAL_PUMP: {
            pumpState = MANUAL_PUMP_OFF;
            stateOfPump = 0;
            break;
        }
        case MANUAL_PUMP_OFF: {
            controlPump();
            if (rcv_cmd_state == ONPUMP && soilMoisture < H_TH) {
                pumpState = MANUAL_PUMP_ON;
                stateOfPump = 1;
                break;
            }
            if (rcv_cmd_mode == AUTO) {
                pumpState = AUTO_PUMP;
                rcv_cmd_state == OFFPUMP;
                break;
            }
            break;
        }
        case MANUAL_PUMP_ON: {
            controlPump();
            if (rcv_cmd_state == OFFPUMP || soilMoisture >= H_TH) {
                pumpState = MANUAL_PUMP_OFF; 
                stateOfPump = 0; 
                break;
            }
            if (rcv_cmd_mode == AUTO) {
                pumpState = AUTO_PUMP;
                rcv_cmd_state == OFFPUMP;
                break;
            }
            break;
        }
        default: {
            break;
        }
    }
}
void updateFromFeed() {
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(500))) {
        if (subscription == &pumpControl) {
            String value1 = (char *)pumpControl.lastread;
            int state = value1.toInt();
            Serial.print("State of Pump from Server: ");
            if (state == 0) {
                rcv_cmd_state = OFFPUMP;
                Serial.println("OFFPUMP");
            }
            else {
                rcv_cmd_state = ONPUMP;
                Serial.println("ONPUMP");
            }
        }
        if (subscription == &modeControl) {
            String value2 = (char *)modeControl.lastread;
            int mode = value2.toInt();
            Serial.print("Mode Control from Server: ");
            if (mode == 0) {
                rcv_cmd_mode = AUTO;
                Serial.println("AUTO");
            }
            else {
                rcv_cmd_mode = MAN;
                Serial.println("MANUAL");
            }
        }
        if (subscription == &lThControl) {
            String value3 = (char *)lThControl.lastread;
            // L_TH = (value3.toInt()) * 4096 / 100;
            L_TH = value3.toInt();
            if (L_TH > H_TH) L_TH = H_TH - 1;
            Serial.print("Low Threshold from Server: ");
            Serial.println(L_TH);
        }
        if (subscription == &hThControl) {
            String value4 = (char *)hThControl.lastread;
            H_TH = value4.toInt();
            if (H_TH < L_TH) H_TH = L_TH + 1;
            Serial.print("High Threshold from Server: ");
            Serial.println(H_TH);
        }
    }
}

// Task Scheduler
Scheduler scheduler;
Task taskReadSensors(10000, TASK_FOREVER, &readSensors);
Task taskUpdateToFeed(10000, TASK_FOREVER, &updateToFeed);
Task taskUpdateToLCD(5, TASK_FOREVER, &updateToLCD);
Task taskUpdateGlobalPump(5, TASK_FOREVER, &globalPump);
Task taskUpdateAutomaticPump(5, TASK_FOREVER, &automaticPump);
Task taskUpdateManualPump(5, TASK_FOREVER, &manualPump);
Task taskUpdateFromFeed(1, TASK_FOREVER, &updateFromFeed);
Task taskConnectMQTT(10, TASK_FOREVER, &connectMQTT);
Task taskCheckButton(1, TASK_FOREVER, &readDebouncedButton);

void setup() {
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(RELAY_PIN, LOW);
    dht20.begin();
    lcd.init();           
    lcd.backlight();
    connectWiFi();
    mqtt.subscribe(&pumpControl);
    mqtt.subscribe(&modeControl);
    mqtt.subscribe(&lThControl);
    mqtt.subscribe(&hThControl);
    connectMQTT();
    scheduler.addTask(taskReadSensors);
    scheduler.addTask(taskUpdateToFeed);
    scheduler.addTask(taskUpdateToLCD);
    scheduler.addTask(taskUpdateGlobalPump);
    scheduler.addTask(taskUpdateAutomaticPump);
    scheduler.addTask(taskUpdateManualPump);
    scheduler.addTask(taskUpdateFromFeed);
    scheduler.addTask(taskConnectMQTT);
    scheduler.addTask(taskCheckButton);
    taskReadSensors.enable();
    taskUpdateToFeed.enable();
    taskUpdateToLCD.enable();
    taskUpdateGlobalPump.enable();
    taskUpdateAutomaticPump.enable();
    taskUpdateManualPump.enable();
    taskUpdateFromFeed.enable();
    taskConnectMQTT.enable();
    taskCheckButton.enable();
}

void loop() {
    scheduler.execute();
}