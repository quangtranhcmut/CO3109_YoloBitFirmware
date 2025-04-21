// #include <Arduino.h>
// #include <Wifi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>

// #define RELAY 25
// const int freq = 5000; //Hz
// const int resolution = 8; //0 -> 16 bits
// const int channel = 0; //0 -> 15 channels
// void setup () {
//   ledcSetup(channel, freq, resolution); //cau hinh kenh PWM channel (0) voi tan so va do phan giai
//   ledcAttachPin(RELAY, channel); //gan kenh PWM channel (0) voi GPIO RELAY (25)
// }
// void loop () {
//   for(int dutyCycle = 0; dutyCycle < 255; dutyCycle++){
//     ledcWrite(channel, dutyCycle);
//     delay(200);
//   }
//   for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
//     ledcWrite(channel, dutyCycle);
//     delay(200);
//   }
// }

// #define INPUT_ADC 34
// int adc_value;
// void setup () {
//   Serial.begin(9600);
//   delay(200);
// }
// void loop () {
//   adc_value = analogRead(INPUT_ADC);
//   Serial.print("Value: "); 
//   Serial.println(adc_value);
//   delay(1000);
// }

// const int sensorPin = 27;   
// const int relayPin = 26;    
// enum State {
//   SENSOR_HIGH_RELAY_OFF,
//   SENSOR_LOW_RELAY_ON,
//   SENSOR_HIGH_RELAY_ON
// };
// unsigned long relayTurnOffTime = 0; 
// volatile State currentState = SENSOR_HIGH_RELAY_OFF; 
// void IRAM_ATTR handleSensorInterrupt() {
//   int sensorState = digitalRead(sensorPin); 
//   if (sensorState == LOW) {
//     currentState = SENSOR_LOW_RELAY_ON; 
//   } 
//   else {
//     currentState = SENSOR_HIGH_RELAY_ON;
//     relayTurnOffTime = millis() + 5000; 
//   }
// }
// void setup() {
//   Serial.begin(9600); 
//   pinMode(sensorPin, INPUT_PULLUP); 
//   pinMode(relayPin, OUTPUT);
//   digitalWrite(relayPin, LOW); 
//   attachInterrupt(digitalPinToInterrupt(sensorPin), handleSensorInterrupt, CHANGE);
// }
// void loop() {
//   switch (currentState) {
//     case SENSOR_HIGH_RELAY_OFF:
//     {
//       digitalWrite(relayPin, LOW);
//       break;
//     }
//     case SENSOR_LOW_RELAY_ON:
//     {
//       digitalWrite(relayPin, HIGH);
//       Serial.println("Sensor State: LOW - Relay ON");
//       break;
//     }
//     case SENSOR_HIGH_RELAY_ON:
//     {
//       digitalWrite(relayPin, HIGH);
//       if (millis() >= relayTurnOffTime) {
//         currentState = SENSOR_HIGH_RELAY_OFF; 
//         Serial.println("Sensor State: HIGH - Relay OFF after 5 seconds");
//       }
//       else {
//         Serial.println("Sensor State: HIGH - Relay ON for remaining time");
//       }
//       break;
//     }
//   }
// }

// const int ledPin = 26;    
// const int buttonPin = 0; 
// typedef enum {
//   STATE_LED_ON,
//   STATE_LED_BLINK_SLOW,
//   STATE_LED_BLINK_FAST,
//   STATE_LED_OFF
// } State;
// State currentState = STATE_LED_ON; // Trạng thái ban đầu
// unsigned long previousMillis = 0;  // Thời gian lưu trữ để so sánh
// const long intervalSlowOn = 2000;  // 2s
// const long intervalSlowOff = 1000; // 1s
// const long intervalFast = 500;     // 0.5s
// bool ledState = LOW;               // Trạng thái đèn LED
// void setup() {
//   pinMode(ledPin, OUTPUT);      
//   pinMode(buttonPin, INPUT_PULLUP);    
//   Serial.begin(9600);       
// }
// void loop() {
//   unsigned long currentMillis = millis(); 
//   if (digitalRead(buttonPin) == LOW) {
//     delay(50); 
//     while (digitalRead(buttonPin) == LOW); // Chờ nút nhả ra
//     // Chuyển trạng thái
//     switch (currentState) {
//       case STATE_LED_ON:
//         currentState = STATE_LED_BLINK_SLOW;
//         break;
//       case STATE_LED_BLINK_SLOW:
//         currentState = STATE_LED_BLINK_FAST;
//         break;
//       case STATE_LED_BLINK_FAST:
//         currentState = STATE_LED_OFF;
//         break;
//       case STATE_LED_OFF:
//         currentState = STATE_LED_ON;
//         break;
//     }
//     Serial.print("Current state: ");
//     Serial.println(currentState);
//   }
//   switch (currentState) {
//     case STATE_LED_ON:
//       digitalWrite(ledPin, HIGH);
//       break;
//     case STATE_LED_BLINK_SLOW:
//       if (ledState == HIGH && currentMillis - previousMillis >= intervalSlowOn) {
//         ledState = LOW;
//         previousMillis = currentMillis;
//       } 
//       else if (ledState == LOW && currentMillis - previousMillis >= intervalSlowOff) {
//         ledState = HIGH;
//         previousMillis = currentMillis;
//       }
//       digitalWrite(ledPin, ledState);
//       break;
//     case STATE_LED_BLINK_FAST:
//       if (currentMillis - previousMillis >= intervalFast) {
//         ledState = !ledState;
//         previousMillis = currentMillis;
//         digitalWrite(ledPin, ledState);
//       }
//       break;
//     case STATE_LED_OFF:
//       digitalWrite(ledPin, LOW);
//       break;
//   }
// }

// #define BOOT_BUTTON 0
// void setup () {
//   Serial.begin(9600);
//   pinMode(BOOT_BUTTON, INPUT_PULLUP);
//   WiFi.mode(WIFI_STA);
//   WiFi.disconnect();
//   Serial.println("Setup done!");
//   delay(2000);
// }
// void loop () {
//   if (digitalRead(BOOT_BUTTON) == LOW) {
//     delay(50);
//     while (digitalRead(BOOT_BUTTON) == LOW);
//     Serial.println("Scan start...");
//     int n = WiFi.scanNetworks();
//     if (n == 0) {
//       Serial.println("No WiFi found!");
//     }
//     else {
//       Serial.println("Found " + String(n) + " WiFi");
//       for(int i = 0; i < n; i++){
//         Serial.println(String(i + 1) + ": " + WiFi.SSID(i)
//                         + "(" + WiFi.RSSI(i) + ")");
//         delay(500);
//       }
//     }
//     Serial.println("Done!");
//   }
// }

// const char SSID [] = "The Sun";
// const char PASSWORD [] = "88888888";
// void initWiFi () {
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(SSID, PASSWORD);
//   Serial.print("Connecting to WiFi...");
//   while (WiFi.status() != WL_CONNECTED){
//     Serial.print(".");
//     delay(1000);
//   }
//   Serial.print(" ");
//   Serial.println(WiFi.localIP());
// }
// void setup () {
//   Serial.begin(9600);
//   initWiFi();
//   Serial.print("RSSI: ");
//   Serial.println(WiFi.RSSI());
// }
// void loop () {
// }

// const char SSID [] = "The Sun";
// const char PASSWORD [] = "88888888";
// IPAddress local_IP(192, 168, 1, 99);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 0, 0);
// IPAddress primaryDNS(8, 8, 8, 8);
// IPAddress secondaryDNS(8, 8, 4, 4);
// void setup () {
//   Serial.begin(9600);
//   if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//     Serial.println("STA failed to configure!");
//   }
//   Serial.print("Connecting to ");
//   Serial.print(SSID);
//   WiFi.begin(SSID, PASSWORD);
//     while (WiFi.status() != WL_CONNECTED){
//       Serial.print(".");
//       delay(500);
//     }
//     Serial.println("");
//     Serial.println("WiFi connected!");
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());
// }
// void loop () {
// }

// const char SSID [] = "The Sun";
// const char PASSWORD [] = "88888888";
// bool printed = true;
// unsigned long previousMillis = 0;
// unsigned long interval = 5000;
// void initWiFi () {
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(SSID, PASSWORD);
//   Serial.print("Connecting to WiFi...");
//   while (WiFi.status() != WL_CONNECTED){
//     Serial.print(".");
//     delay(1000);
//   }
//   Serial.println("");
//   Serial.print("IP: ");
//   Serial.println(WiFi.localIP());
// }
// void setup () {
//   Serial.begin(9600);
//   initWiFi();
//   Serial.print("RSSI: ");
//   Serial.println(WiFi.RSSI());
// }
// void loop () {
//   unsigned long currentMillis = millis();
//   if (WiFi.status() != WL_CONNECTED && (currentMillis - previousMillis >= interval)) {
//     Serial.println("Reconneting...");
//     WiFi.disconnect();
//     WiFi.reconnect();
//     previousMillis = currentMillis;
//   }
//   if (WiFi.status() == WL_CONNECTED && (currentMillis - previousMillis >= 2 * interval)){
//     Serial.print("RSSI: ");
//     Serial.println(WiFi.RSSI());
//     previousMillis = currentMillis;
//   }
// }


// const char SSID [] = "....";
// const char PASSWORD [] = "11111111";
// WiFiServer server(80);
// String header;

// const int LED1 = 26;
// const int LED2 = 27;
// String LED1_state = "OFF";
// String LED2_state = "OFF";

// unsigned long currentTime = millis();
// unsigned long previousTime = 0; 
// const long timeoutTime = 2000;

// void initWiFi () {
//   Serial.print("Connecting to ");
//   Serial.print(SSID);
//   WiFi.begin(SSID, PASSWORD);
//   while (WiFi.status() != WL_CONNECTED){
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println("");
//   Serial.println("WiFi connected!");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

// void setup () {
//   Serial.begin(9600);
//   pinMode(LED1, OUTPUT);
//   pinMode(LED2, OUTPUT);
//   digitalWrite(LED1, LOW);
//   digitalWrite(LED2, LOW);

//   initWiFi();
//   server.begin();
// }
// void loop () {
//   WiFiClient client = server.available();
//   if (client) {
//     Serial.println("New Client.");
//     String currentLine = "";
//     while (client.connected()) {
//       if (client.available()) {
//         char c = client.read(); 
//         Serial.write(c);   
//         header += c;
//         if (c == '\n') {
//           if (currentLine.length() == 0)
//           {
//               client.println("HTTP/1.1 200 OK");
//               client.println("Content-type:text/html");
//               client.println("Connection: close");
//               client.println();

//               if (header.indexOf("GET /led1/on") >= 0)
//               {
//                   Serial.println("LED 1 on");
//                   LED1_state = "ON";
//                   digitalWrite(LED1, HIGH);
//               }
//               else if (header.indexOf("GET /led1/off") >= 0)
//               {
//                   Serial.println("LED 2 off");
//                   LED1_state = "OFF";
//                   digitalWrite(LED1, LOW);
//               }
//               else if (header.indexOf("GET /led2/on") >= 0)
//               {
//                   Serial.println("LED 2 on");
//                   LED2_state = "ON";
//                   digitalWrite(LED2, HIGH);
//               }
//               else if (header.indexOf("GET /led2/off") >= 0)
//               {
//                   Serial.println("LED 2 off");
//                   LED2_state = "OFF";
//                   digitalWrite(LED2, LOW);
//               }

//               client.println("<!DOCTYPE html><html>");
//               client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
//               client.println("<link rel=\"icon\" href=\"data:,\">");
//               client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
//               client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
//               client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
//               client.println(".button2 {background-color: #555555;}</style></head>");
//               client.println("<body><h1>ESP32 Web Server</h1>");

//               client.println("<p>LED 1 - State " + LED1_state + "</p>");
//               if (LED1_state == "OFF")
//               {
//                   client.println("<p><a href=\"/led1/on\"><button class=\"button\">ON</button></a></p>");
//               }
//               else
//               {
//                   client.println("<p><a href=\"/led1/off\"><button class=\"button button2\">OFF</button></a></p>");
//               }

//               client.println("<p>LED 2 - State " + LED2_state + "</p>");
//               if (LED2_state == "OFF")
//               {
//                   client.println("<p><a href=\"/led2/on\"><button class=\"button\">ON</button></a></p>");
//               }
//               else
//               {
//                   client.println("<p><a href=\"/led2/off\"><button class=\"button button2\">OFF</button></a></p>");
//               }
//               client.println("</body></html>");
//               client.println();
//               break;
//           }
//           else
//           { 
//               currentLine = "";
//           }
//         }
//         else if (c != '\r')
//         {                     // if you got anything else but a carriage return character,
//             currentLine += c; // add it to the end of the currentLine
//         }
//       }
//     }
//     header = ""; 
//     client.stop();
//     Serial.println("Client disconnected.");
//     Serial.println("");
//   }
// }


// const int ctSensorPin = 34;  // Chân ADC của ESP32 mà cảm biến CT được kết nối đến
// void setup() {
//   Serial.begin(9600);  // Khởi động Serial Monitor
// }
// void loop() {
//   // Đọc giá trị ADC từ chân cảm biến
//   int sensorValue = analogRead(ctSensorPin);
//   // Chuyển đổi giá trị ADC thành millivolts (mV)
//   float voltage = (sensorValue * 3300.0) / 4095.0;
//   // In giá trị millivolts ra Serial Monitor
//   Serial.print("Voltage: ");
//   Serial.print(voltage);
//   Serial.println(" mV");
//   delay(250);  // Đợi 1 giây trước khi đọc lại giá trị
// }


// #include "ADS1X15.h"
// #include <LiquidCrystal_I2C.h>
// ADS1115 ADS(0x48);
// LiquidCrystal_I2C lcd(0x25, 16, 2);

// void doc_ADS()
// {
//   	ADS.setGain(1);

//   	int16_t val_0 = ADS.readADC(0);  
//   	int16_t val_1 = ADS.readADC(1);  
//   	int16_t val_2 = ADS.readADC(2);  
//   	int16_t val_3 = ADS.readADC(3);  

//   	float f = ADS.toVoltage(1);  

//     float may1 = val_0 * f;
//     float may2 = val_1 * f;
//     float may3 = val_2 * f;
//     float may4 = val_3 * f;
	
// 	float current = ( may1 / sqrt(2) ) * 30;

//   	Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.println(val_0 * f, 3);
//   	// Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t'); Serial.println(val_1 * f, 3);
//   	// Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t'); Serial.println(val_2 * f, 3);
//   	// Serial.print("\tAnalog3: "); Serial.print(val_3); Serial.print('\t'); Serial.println(val_3 * f, 3);
//   	Serial.println();
// }

// void setup() {
//     Wire.begin();
//     Serial.begin(9600);
//     Serial.print("ADS1X15_LIB_VERSION: ");
//     Serial.println(ADS1X15_LIB_VERSION);
//     lcd.begin(16, 2);
//     lcd.backlight();
// 	pinMode(13, OUTPUT);  
// 	digitalWrite(13, HIGH);
// }
// int count = 0;
// float sum_vol = 0.0;
// float sum_cur = 0.0;
// float maxvol = 0.0;
// void loop() {
// 	if (count < 100) {
// 		// float voltage = ADS.readADC(0) * ADS.toVoltage(1);
// 		// sum_vol += voltage;
// 		// sum_cur += (voltage / sqrt(2)) * 30;

// 		float voltage = ADS.readADC(0) * ADS.toVoltage(1);
// 		if (voltage >= maxvol) {
// 			maxvol = voltage;
// 		}
		
// 		count++;
// 	}
// 	else {
// 		// float avr_vol = sum_vol / 100;
// 		// float avr_cur = sum_cur / 100;
// 		// lcd.setCursor(0, 0);
// 		// lcd.print("A0: ");
// 		// lcd.print(avr_vol, 3); 
// 		// lcd.setCursor(0, 6);
// 		// lcd.print("I0: ");
// 		// lcd.print(avr_cur, 3); 
// 		// sum_vol = 0.0;
// 		// sum_cur = 0.0;

// 		lcd.setCursor(0, 0);
// 		lcd.print("A0: ");
// 		lcd.print(maxvol, 3); 
// 		lcd.setCursor(0, 6);
// 		lcd.print("I0: ");
// 		lcd.print((maxvol / sqrt(2)) * 30, 3); 
// 		maxvol = 0.0;
		
// 		count = 0;
// 		delay(500);
// 	}
// }



// #include <Wire.h>
// #include <Adafruit_BusIO_Register.h>
// void setup() {
//   Serial.begin(9600);
//   Wire.begin(); // Khởi tạo I2C, sử dụng SDA và SCL mặc định của ESP32 (thường là GPIO21 và GPIO22)

//   Serial.println("Scanning I2C devices...");
// }

// void loop() {
//   byte error, address;
//   int nDevices = 0;

//   for (address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println(" !");
//       nDevices++;
//     } else if (error == 4) {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }

//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000); // Quét lại sau 5 giây
// }


// #include <WiFi.h>
// #include "Adafruit_MQTT.h"
// #include "Adafruit_MQTT_Client.h"

// // WiFi credentials
// #define WIFI_SSID "Lmao"
// #define WIFI_PASSWORD "khongdungchua"

// // Adafruit IO credentials
// #define AIO_USERNAME "tranhaithaoquang"   // Tên tài khoản Adafruit IO
// #define AIO_KEY "aio_RFsX33n42uI8sWpL5TQlozLSOPUl"    // API Key của bạn
// #define FEED_ID "bcc-temp"      // ID của feed

// // MQTT Server Configuration
// #define AIO_SERVER "io.adafruit.com"
// #define AIO_SERVERPORT 1883

// WiFiClient client;
// Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// // Feed để gửi dữ liệu
// Adafruit_MQTT_Publish moistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" FEED_ID);

// void connectWiFi() {
//     Serial.print("Connecting to WiFi...");
//     WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//     while (WiFi.status() != WL_CONNECTED) {
//         Serial.print(".");
//         delay(500);
//     }
//     Serial.println("Connected!");
// }

// void connectMQTT() {
//     Serial.print("Connecting to Adafruit IO...");
//     while (!mqtt.connected()) {
//         if (mqtt.connect()) {
//             Serial.println("Connected!");
//         } else {
//             Serial.print("Failed, rc=");
//             Serial.print(mqtt.connectErrorString(mqtt.connect()));
//             Serial.println(" Retrying in 5 seconds...");
//             delay(5000);
//         }
//     }
// }

// void setup() {
//     Serial.begin(9600);
//     connectWiFi();
//     connectMQTT();
// }

// void loop() {
//     if (!mqtt.connected()) {
//         connectMQTT();
//     }

//     // Tạo giá trị giả lập cho độ ẩm đất
//     int randomMoisture = random(300, 800);
    
//     Serial.print("Sending moisture value: ");
//     Serial.println(randomMoisture);

//     // Gửi dữ liệu lên Adafruit IO
//     if (!moistureFeed.publish(randomMoisture)) {
//         Serial.println("Failed to publish!");
//     }

//     delay(5000); // Gửi dữ liệu mỗi 5 giây
// }



// #include <WiFi.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_MQTT.h>
// #include <Adafruit_MQTT_Client.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_AHTX0.h>
// #include <TaskScheduler.h>
// #include <Ethernet.h>
// #include <UIPEthernet.h>

// // WiFi & Ethernet Configuration
// #define WIFI_SSID "your_wifi_ssid"
// #define WIFI_PASSWORD "your_wifi_password"
// byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// EthernetClient ethClient;
// WiFiClient wifiClient;
// Client *activeClient;

// // Adafruit IO Credentials
// #define AIO_USERNAME "your_username"
// #define AIO_KEY "your_adafruit_key"
// #define FEED_MOISTURE "moisture_sensor"
// #define FEED_TEMP "temperature"
// #define FEED_HUMIDITY "humidity"
// #define FEED_PUMP "pump"

// // MQTT Configuration
// Adafruit_MQTT_Client mqtt(&wifiClient, "io.adafruit.com", 1883, AIO_USERNAME, AIO_KEY);
// Adafruit_MQTT_Publish moistureFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_MOISTURE);
// Adafruit_MQTT_Publish tempFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_TEMP);
// Adafruit_MQTT_Publish humidityFeed(&mqtt, AIO_USERNAME "/feeds/" FEED_HUMIDITY);
// Adafruit_MQTT_Subscribe pumpControl(&mqtt, AIO_USERNAME "/feeds/" FEED_PUMP);

// // Sensor & LCD Configuration
// #define SOIL_SENSOR_PIN 36
// #define RELAY_PIN 5
// #define OLED_RESET -1
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_AHTX0 aht;

// // System Variables
// enum PumpState { OFF, AUTO_ON, MANUAL_ON };
// PumpState pumpState = OFF;
// int soilMoisture = 0;
// float temperature = 0;
// float humidity = 0;

// // Thresholds
// #define MOISTURE_THRESHOLD 500  // Ngưỡng bật bơm

// // Function Prototypes
// void readSensors();
// void controlPump();
// void updateAdafruitIO();
// void checkPumpCommand();
// void updateLCD();

// // Task Scheduler
// Scheduler scheduler;
// Task taskReadSensors(5000, TASK_FOREVER, &readSensors);
// Task taskControlPump(1000, TASK_FOREVER, &controlPump);
// Task taskUpdateAdafruitIO(10000, TASK_FOREVER, &updateAdafruitIO);
// Task taskCheckPumpCommand(2000, TASK_FOREVER, &checkPumpCommand);
// Task taskUpdateLCD(1000, TASK_FOREVER, &updateLCD);

// void connectNetwork() {
//     Serial.print("Checking Ethernet...");
//     if (Ethernet.begin(mac) == 1) {
//         Serial.println("Connected via Ethernet");
//         activeClient = &ethClient;
//     } else {
//         Serial.println("Using WiFi...");
//         WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//         while (WiFi.status() != WL_CONNECTED) {
//             Serial.print(".");
//             delay(500);
//         }
//         Serial.println("WiFi Connected");
//         activeClient = &wifiClient;
//     }
//     mqtt = Adafruit_MQTT_Client(activeClient, "io.adafruit.com", 1883, AIO_USERNAME, AIO_KEY);
// }

// void connectMQTT() {
//     Serial.print("Connecting to Adafruit IO...");
//     while (!mqtt.connected()) {
//         if (mqtt.connect()) {
//             Serial.println("Connected!");
//             mqtt.subscribe(&pumpControl);
//         } else {
//             Serial.println("MQTT failed, retrying...");
//             delay(5000);
//         }
//     }
// }

// void readSensors() {
//     sensors_event_t humidityEvent, tempEvent;
//     aht.getEvent(&humidityEvent, &tempEvent);

//     soilMoisture = analogRead(SOIL_SENSOR_PIN);
//     temperature = tempEvent.temperature;
//     humidity = humidityEvent.relative_humidity;

//     Serial.printf("Moisture: %d, Temp: %.2fC, Humidity: %.2f%%\n", soilMoisture, temperature, humidity);
// }

// void controlPump() {
//     if (pumpState == MANUAL_ON) {
//         digitalWrite(RELAY_PIN, HIGH);
//     } else if (pumpState == AUTO_ON && soilMoisture < MOISTURE_THRESHOLD) {
//         digitalWrite(RELAY_PIN, HIGH);
//     } else {
//         digitalWrite(RELAY_PIN, LOW);
//         pumpState = OFF;
//     }
// }

// void updateAdafruitIO() {
//     if (!mqtt.connected()) connectMQTT();
//     mqtt.processPackets(100);
    
//     moistureFeed.publish(soilMoisture);
//     tempFeed.publish(temperature);
//     humidityFeed.publish(humidity);
    
//     Serial.println("Updated Adafruit IO!");
// }

// void checkPumpCommand() {
//     Adafruit_MQTT_Subscribe *subscription;
//     while ((subscription = mqtt.readSubscription(100))) {
//         if (subscription == &pumpControl) {
//             String command = (char *)pumpControl.lastread;
//             if (command == "ON") {
//                 pumpState = MANUAL_ON;
//                 Serial.println("Pump set to MANUAL ON");
//             } else if (command == "OFF") {
//                 pumpState = OFF;
//                 Serial.println("Pump set to OFF");
//             }
//         }
//     }
// }

// void updateLCD() {
//     display.clearDisplay();
//     display.setTextSize(1);
//     display.setTextColor(SSD1306_WHITE);
//     display.setCursor(0, 0);
//     display.printf("Temp: %.1fC\nHumidity: %.1f%%\nSoil: %d\nPump: %s",
//                    temperature, humidity, soilMoisture, (pumpState == OFF ? "OFF" : "ON"));
//     display.display();
// }

// void setup() {
//     Serial.begin(9600);
//     connectNetwork();
//     connectMQTT();

//     pinMode(RELAY_PIN, OUTPUT);
//     digitalWrite(RELAY_PIN, LOW);
    
//     if (!aht.begin()) {
//         Serial.println("Không tìm thấy cảm biến AHT20!");
//         while (1);
//     }
//     Serial.println("Cảm biến AHT20 đã sẵn sàng!");
    
//     display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
//     display.clearDisplay();
    
//     scheduler.addTask(taskReadSensors);
//     scheduler.addTask(taskControlPump);
//     scheduler.addTask(taskUpdateAdafruitIO);
//     scheduler.addTask(taskCheckPumpCommand);
//     scheduler.addTask(taskUpdateLCD);
    
//     taskReadSensors.enable();
//     taskControlPump.enable();
//     taskUpdateAdafruitIO.enable();
//     taskCheckPumpCommand.enable();
//     taskUpdateLCD.enable();
// }

// void loop() {
//     scheduler.execute();
// }


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
            L_TH = (value3.toInt()) * 4096 / 100;
            if (L_TH > H_TH) L_TH = H_TH - 1;
            Serial.print("Low Threshold from Server: ");
            Serial.println(L_TH);
        }
        if (subscription == &hThControl) {
            String value4 = (char *)hThControl.lastread;
            H_TH = (value4.toInt()) * 4096 / 100;
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