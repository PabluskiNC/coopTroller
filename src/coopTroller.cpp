/*
Arduino Chicken Coop Controller
Based loosely on code by Will Vincent <will@willvincent.com>
This Sketch is designed to work on a ESP-8266 MUANODE 0.9 board
*/

#include <Arduino.h>
//#include <pins_arduino.h>
#include "options.h"
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <Wire.h>                 // https://www.arduino.cc/en/Reference/Wire
//#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266mDNS.h>
//#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient
#include <Bounce2.h>              // https://github.com/thomasfredericks/Bounce2
#include <RTClib.h>               // fork of https://github.com/adafruit/RTClib with getTemperature function added
#ifdef LCD_DISPLAY
  #include <LiquidCrystal_I2C.h>
#endif
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson
#include "pins.h"                 // holds pin definitions

#define CoopTrollerVersion "3.04h"

// MQTT Subscription Channels
#define sTime    "time/beacon"
#define sSunRise "sun/rise"
#define sSunSet  "sun/set"
#define sRemote  "coop/remotetrigger"

// MQTT Publish Channels
//#define pLight  "coop/brightness"
#define pJstatus "coop/jstatus"
#define pStatus  "coop/status"

// DOOR motion. Relays are active on low
#define GO     0
#define STOP   1

// Misc Settings
const unsigned long millisPerDay    = 86400000; // Milliseconds per day
const unsigned long lightReadRate   =    30000; // How often to read light level (30 sec)
const unsigned long remoteOverride  =   600000; // Length of time to lockout readings. (10 min)
const unsigned long publishInterval =   600000; // How often to publish light sensor readings. (10 min)
const unsigned long maxMotorOn      =     6000; // Maximum time for the motor to be on (6 seconds)
const unsigned long lcdUpdateRate   =     5000; // LCD update rate (5 seconds)

// Night time lockout to prevent reaction to light sensor readings if an exterior light source causes
// a reading otherwise bright enough to activate the interior light and/or door.
const boolean       nightLock      =      true; // Enable night time lockout

/*************************************************
       DO   NOT   EDIT   BELOW   THIS   LINE
 *************************************************/

// Runtime variables
int           nightLockStart     =     22; // Hour (in 24hr time) to initiate night time lockout (10pm)
int           nightLockEnd       =      4; // Hour (in 24hr time) to end night time lockout (4am)
unsigned long lastDebounce       =      0;
unsigned long lastLightRead      =      0;
unsigned long lastRTCSync        =      0;
unsigned long remoteLockStart    =      0;
unsigned long motorRunning       =      0;  // how long has the motor been on
unsigned long motorTimeOut       = 300000;  // Time to wait before running motor again (5 minutes)
unsigned long lastMotorRun       =      0;  // last time the motor was activated
unsigned long lastLcdUpdate      =      0;  // last time LCD was updated
String        doorState          =  "Uninitialzed"; // Values will be one of: closed, closing, open, opening, unknown
String        doorStatePrev      =     "";
int           brightness         =      0;
int           openBright         =     40; // brightness to wait until opening the door
int           closeBright        =     25; // brightness to wait until closing the door
int           doorTopVal         =      0;
int           doorTopVal2        =      0;
int           doorTopState       =      0;
int           doorTopPrev        =      0;
int           doorBottomVal      =      0;
int           doorBottomVal2     =      0;
int           doorBottomState    =      0;
int           doorBottomPrev     =      0;
float         tempC              =      0;  // temperature
//uint32_t      bootTime           =      0;
char          mqtt_msg_buf[100];   // Max MQTT incoming message size
DateTime      now;
boolean wifiConnected = false;

#ifdef LCD_DISPLAY
char lcd_buf[21]; // LCD cols + 1 for null character
#endif

//define your default values here, if there are different values in config.json, they are overwritten.
char mClientID[30];
char mqtt_server[40];
char mqtt_port[6] = "1883";
//flag for saving data
bool shouldSaveConfig = false;

// Define the hardware components
// Real Time Clock
RTC_DS3231 RTC;
// ESP8266 instance
EspClass esp;
// WiFi instance
WiFiClient espClient;

// LCD (if enabled)
#ifdef LCD_DISPLAY
LiquidCrystal_I2C lcd(0x27, 20, 4);   // i2c address x27, 20 cols x 4 rows
#endif

// MQTT
PubSubClient mqtt(espClient);

// Setup switch debounce software
Bounce debounceTop = Bounce();
Bounce debounceBot = Bounce();

/**
 * WiFi Manager callback notifying us of the need to save config
 */
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/**
 * Manage WIFI connection
 */
void wifiCb(WiFiEvent_t event) {
  if (Debugging) {
    Serial.printf("\n[WiFi-event] event: %d\n", event);
  }

  switch(event) {
      case WIFI_EVENT_STAMODE_GOT_IP:
         if (Debugging) {
            Serial.print("WiFi IP: ");
            Serial.println(WiFi.localIP());
         }
         mqtt.connect(mClientID);
         if ( !mqtt.connected() ) {
            delay(100);
            mqtt.connect(mClientID);
         }
         wifiConnected = true;
         break;
     case WIFI_EVENT_STAMODE_DISCONNECTED:
         if (Debugging) {
            Serial.println("WiFi lost connection");
         }
         wifiConnected = false;
         mqtt.disconnect();
         break;
    }
}

/**
 * JSON Status Send
 */
int jstatusSend() {
   char buf[121]; // 128 bytes is the maximum MQTT message length allowed in PubSubClient
   DateTime now = RTC.now();
   // DynamicJsonBuffer jsonBuffer;
   StaticJsonBuffer<200> jsonBuffer;
   JsonArray& mqttRoot  = jsonBuffer.createArray();
   JsonObject& mqttJson = mqttRoot.createNestedObject();
   mqttJson["ID"]       = mClientID;
   mqttJson["tstamp"]   = now.unixtime();
   JsonObject& data     = mqttJson.createNestedObject("data");
   data["Br"]           = brightness;
   data["Dr"]           = doorState;
   data["Tp"]           = doorTopState;
   data["Bt"]           = doorBottomState;
   data["TC"]           = tempC;

   mqttJson.printTo(buf,sizeof(buf));
   if (Debugging) {
     Serial.println(buf);
   }
   return mqtt.publish(pJstatus,buf);
}

/**
 * MQTT Connection event handler.
 *
 * Subscribes to desired channels
 */
void mqttSubscribe() {
  if (wifiConnected) {
     if (Debugging) {
       Serial.println("MQTT Subscribing");
     }
     // Subscribe to time beacon channel to keep RTC up to date.
     mqtt.subscribe(sTime, 0); // QoS 0 means no verification, 1 means verificaton

     // Subscribe to remote trigger channel to allow remote control of chicken coop
     mqtt.subscribe(sRemote, 1);

     // Subscribe to sunrise/set updates
     mqtt.subscribe(sSunRise, 1);
     mqtt.subscribe(sSunSet, 1);

     // Publish that we're online!
     mqtt.publish(pStatus, "mqttOnline");
  } else {
     if (Debugging) {
       Serial.println("MQTT NOT Subscribed because WIFI is not connected");
     }
  }
}

/**
 * Handle incoming MQTT messages.
 *
 * This allows us to remotely trigger events via WIFI!
 */
void mqttData(char* topic, byte* payload, unsigned int plen) {

  // Copy the payload to the MQTT message buffer
  if( plen > sizeof(mqtt_msg_buf)) {  // buffer is only 100 bytes long
    plen = sizeof(mqtt_msg_buf);
  }
  memset(mqtt_msg_buf,'\0',plen+1);
  memcpy(mqtt_msg_buf,payload,plen);
  String data  = String((char *) mqtt_msg_buf);

  if(Debugging) {
     Serial.print("mqttTopic*Len*Data: |");
     Serial.print(topic);
     Serial.print("| * |");
     Serial.print(plen);
     Serial.print("| * |");
     Serial.print(data);
     Serial.println("|");
  }

  if (strcmp(topic,sRemote)==0) {
    // If door movement is triggered, toggle door state to
    // opening or closing based on current state.
    // If door is currently moving, the trigger is ignored.
    if (data == "version") {  // get the software version loaded
      char buf[100];
      String pubString;
      pubString="Version: " + String(CoopTrollerVersion) + ", Compiled: " + __DATE__ + " " + __TIME__;
      pubString.toCharArray(buf, pubString.length()+1);
      if (wifiConnected) {
         mqtt.publish(pStatus, buf);
      }
    }

    if (data == "status") {  // get a general status ... (obviously WiFi and MQTT are ok)
      char buf[100];
      String pubString;
      pubString="Millis:"+String(millis())+", Door:"+String(doorState)+", Top:"+String(doorTopState)+", Bot:"+String(doorBottomState)+", Light:"+String(brightness)+", Temp:"+String(RTC.getTemperature());
      pubString.toCharArray(buf, pubString.length()+1);
      if (wifiConnected) {
         mqtt.publish(pStatus, buf);
      }
    }

    if (data == "jstatus") { // JSON status
       jstatusSend();
    }

    if (data == "unlock") {  // this will immediately put the system back into normal operation (i.e., using the light sensor)
      remoteLockStart = 0;
      if (wifiConnected) {
        mqtt.publish(pStatus, "remotetrigger|unlock");
      }
      if(Debugging) {
        Serial.println("remoteLockStart unset");
      }
    }
     if (data == "reboot") {
      if (wifiConnected) {
        mqtt.publish(pStatus, "remotetrigger|reboot");
      }
      ESP.restart();
    }
    if (data == "debug") {
      Debugging = 1 - Debugging;
      Serial.print("Debugging now: ");
      Serial.println(Debugging);
    }
    if (data == "door") {
      if(Debugging) {
        Serial.println("remoteLockStart set");
      }
      if (doorState == "open") {
        doorState = "closing";
        remoteLockStart = millis();
        mqtt.publish(pStatus, "remotetrigger|door closing");
      }
      else if (doorState == "closed") {
        doorState = "opening";
        remoteLockStart = millis();
        mqtt.publish(pStatus, "remotetrigger|door opening");
      }
    }
  }

  // Adjust sunrise/set times for nightlock
  if (strcmp(topic,sSunRise)==0) {
    nightLockEnd = atoi(data.c_str());
    if (Debugging) {
      Serial.print("Night lock end updated to: ");
      Serial.println(nightLockEnd);
    }
    mqtt.publish(pStatus, "sunrise");
  }

  if (strcmp(topic,sSunSet)==0) {
    nightLockStart = atoi(data.c_str());
    if (Debugging) {
      Serial.print("Night lock start updated to: ");
      Serial.println(nightLockStart);
    }
    mqtt.publish(pStatus, "sunset");
  }

  // Sync RTC to time beacon once/day
  if (strcmp(topic,sTime)==0) {
    if (lastRTCSync == 0 || ((unsigned long)(millis() - lastRTCSync) > millisPerDay)) {
      RTC.adjust(strtoul(data.c_str(), NULL, 0));
      mqtt.publish(pStatus, "time beacon");
      lastRTCSync = millis();
      if (Debugging) {
        now = RTC.now();
        char dateStr[11];
        char timeStr[9];

        sprintf(dateStr, "%02d/%02d/%04d", now.month(), now.day(), now.year());
        int hr = now.hour();
        boolean ampm = false;
        if (hr > 12) {
          hr = hr - 12;
          ampm = true;
        }
        else if (hr == 12) {
          ampm = true;
        }
        else if (hr == 0) {
          hr = 12;
        }
        sprintf(timeStr, "%02d:%02d:%02d", hr, now.minute(), now.second());
        Serial.println("RTC Updated:");
        Serial.print(dateStr);
        Serial.print(" ");
        Serial.print(timeStr);
        if (ampm) {
          Serial.println("pm");
        }
        else {
          Serial.println("am");
        }

      }
    }
  }
}

/**
 * Handle movement of the door
 */
void doorMove() {

  if (doorState != "halted" ) {
    if(motorRunning > 0) {

      if (motorRunning + maxMotorOn < millis()){
        digitalWrite(doorClose, STOP);
        digitalWrite(doorOpen, STOP);

        if (Debugging) {
          Serial.println("Motor on too long - Halting!");
        }

        if (doorStatePrev != doorState && wifiConnected) {
          mqtt.publish(pStatus, "door|halted");
        }
        //motorHalted = 1;
        doorState="halted";
       }
    }

    doorStatePrev = doorState;
    if (doorState == "closed" || doorState == "closing") {
      if (doorBottomState != 0) {
        // Door isn't closed, run motor until it is.
        if(motorRunning == 0) {  // start your engines
          digitalWrite(doorClose, GO);
          digitalWrite(doorOpen, STOP);
          motorRunning = millis();
          if (Debugging) {
            Serial.printf("Motor CLOSE ON: %i\n", motorRunning);
          }
        }
      }
      else {
        if (motorRunning > 0) {
          // Door is closed, stop motor
          digitalWrite(doorClose, STOP);
          digitalWrite(doorOpen, STOP);
          doorState = "closed";
          motorRunning = 0; // stop the motor running counter
          if (doorStatePrev != doorState && wifiConnected) {
            mqtt.publish(pStatus, "door|closed");
          }
          if (Debugging) {
            Serial.println("Motor CLOSE OFF");
          }
        }
      }
    }
    if (doorState == "open" || doorState == "opening") {
      if (doorTopState != 0) {
        if(motorRunning == 0) {  // start your engines
          // Door isn't open, run motor until it is.
          digitalWrite(doorClose, STOP);
          digitalWrite(doorOpen, GO);        motorRunning = millis();
          if (Debugging) {
            Serial.printf("Motor OPEN ON: %i \n",motorRunning);
          }
        }
      }
      else {
        // Door is open, stop motor.
        if (motorRunning > 0) {
          digitalWrite(doorClose, STOP);
          digitalWrite(doorOpen, STOP);
          doorState = "open";
          motorRunning = 0; // stop the motor running counter
          if (doorStatePrev != doorState && wifiConnected) {
            mqtt.publish(pStatus, "door|open");
          }
          if (Debugging) {
            Serial.println("Motor OPEN OFF");
          }
        }
      }
    }
  }
}

/**
 * Read current sensor data
 */
void readSensors() {
  if (lastLightRead == 0 || (unsigned long)millis() - lastLightRead > lightReadRate) {
    // Read light sensor and convert to brightness percentage
    brightness = analogRead(lightSense);
    brightness = map(brightness, 0, 1023, 0, 100);  // Remap value to a 0-100 scale
    brightness = constrain(brightness, 0, 100);     // constrain value to 0-100 scale
    lastLightRead = millis();

    tempC=RTC.getTemperature(); // get the temperature from the RTC chip

    // Send status msg
    jstatusSend();
    if (Debugging) {
        Serial.print("MQTT State: ");
        Serial.println(mqtt.state());
        Serial.print("Wifi State: ");
        Serial.println(WiFi.status());
    }
    if (Debugging) {
      now = RTC.now();
      Serial.printf("Time:%02d:%02d Light:%03d\n",now.hour(),now.minute(),brightness);
    }

  }
}

/**
 * Respond to updated sensor data.
 */
void handleSensorReadings() {

  // Light based reactions
  // ---------------------

  // Fetch current time from RTC

  now = RTC.now();
  // If nightlock is enabled, and we are within the designated time period, simply
  // ensure door is closed.
  //if(Debugging) {
  //  Serial.printf("Nightlock: %i, Time: %0d:%0d\n",nightLock, now.hour(),now.minute());
  //}
  if (nightLock && ( (now.hour() >= nightLockStart || now.hour() <= nightLockEnd)) ) {

    // Close door if it is open
    if (doorState == "open" || doorState == "unknown") {
      if (Debugging) {
        Serial.println("NIGHTLOCK ENABLED: Closing door.");
      }
      doorState = "closing";
      if (wifiConnected) {
        mqtt.publish(pStatus, "door|closing");
      }
    }
  }
  // Otherwise, handle brightness level based reactions

  // NOTE: We need a bit of a gap between these thresholds to prevent
  // bouncing if light readings fluctuate by a percentage or two.
  else {
    // Open door when brightness level is greater than 5%
    if (brightness >= openBright) {
      if (doorState == "closed" || doorState == "unknown") {
        if (Debugging) {
          Serial.println("Opening door.");
        }
        doorState = "opening";
        if (wifiConnected) {
          mqtt.publish(pStatus, "door|opening");
        }
      }
    }
    // Otherwise, close door when light level falls below 2%.
    else if (brightness < closeBright) {
      if (doorState == "open" || doorState == "unknown") {
        if (Debugging) {
          Serial.println("Closing door.");
        }
        doorState = "closing";
        if (wifiConnected) {
          mqtt.publish(pStatus, "door|closing");
        }
      }
    }
  }
}

int stringToNumber(String thisString) {
  int i, value, length;
  length = thisString.length();
  char blah[(length + 1)];
  for (i = 0; i < length; i++) {
    blah[i] = thisString.charAt(i);
  }
  blah[i] = 0;
  value = atoi(blah);
  return value;
}

#ifdef LCD_DISPLAY
void lcdUpdate() {
  //            1111111111
  //  01234567890123456789
  // +--------------------+
  // |I:xxx.xxx.xxx.xxx   | 0
  // |T:xx.xx B:xxx C:xx  | 1
  // |D:xxxxxxxx L:xx-xx  | 2
  // |MQ:xx T:x B:x       | 3
  // +--------------------+

  if (lastLcdUpdate == 0 || (unsigned long)millis() - lastLcdUpdate > lcdUpdateRate) {
    lastLcdUpdate = millis();
    if (wifiConnected) {
      int a1 = WiFi.localIP()[0];
      int a2 = WiFi.localIP()[1];
      int a3 = WiFi.localIP()[2];
      int a4 = WiFi.localIP()[3];
      snprintf(lcd_buf,21,"I:%03i.%03i.%03i.%03i  ",a1,a2,a3,a4);
    } else {
      snprintf(lcd_buf,21,"I:Wifi Unconnected  ");
    }
    lcd.setCursor(0,0);
    lcd.print(lcd_buf);
    now = RTC.now();
    snprintf(lcd_buf,21,"T:%02d.%02d B:%03d C:%02i",now.hour(),now.minute(),brightness,int(tempC));
    lcd.setCursor(0,1);
    lcd.print(lcd_buf);
    String NLS;
    NLS = "N";
    if(nightLock) {
      NLS = String(nightLockStart)+"-"+String(nightLockEnd);
    }
    snprintf(lcd_buf,21,"D:%-8s L:%1s",doorState.c_str(),NLS.c_str());
    lcd.setCursor(0,2);
    lcd.print(lcd_buf);
    int mqs = mqtt.state();
    snprintf(lcd_buf,21,"MQ:%02i T:%1i B:%1i   ",mqs,doorTopState,doorBottomState);
    lcd.setCursor(0,3);
    lcd.print(lcd_buf);
  }
}
#endif

void WiFiConfig(int reset_config) {

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  //reset settings - for testing
  if (reset_config) {
     wifiManager.resetSettings();
  }

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
}
/**
 * Initialization on startup.
 */
void setup() {
  if (Debugging) {
    Serial.begin(115200);
    Serial.println("Initialising...");
    Serial.println("Setting motor status to STOP");
  }
  #ifdef DEBUG_ESP_CORE
  Serial.setDebugOutput(true);
  #endif

  digitalWrite(doorOpen, STOP);  // Set the moto control relays
  digitalWrite(doorClose, STOP);
  pinMode(doorOpen, OUTPUT);
  pinMode(doorClose, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT);

  // Define the top & bottom sensors
  pinMode(doorTop, INPUT_PULLUP);
  debounceTop.attach(doorTop);
  debounceTop.interval(5);  //inteval in ms
  pinMode(doorBottom, INPUT_PULLUP);
  debounceBot.attach(doorBottom);
  debounceBot.interval(5);  //inteval in ms

  // Again, just in case
  digitalWrite(doorOpen, STOP);
  digitalWrite(doorClose, STOP);

  // unique client name for MQTT
  String macAd = "coopTroller-" + WiFi.macAddress();
  macAd.replace(":",""); // get rid of the colons
  macAd.toCharArray(mClientID, sizeof(mClientID));

  // Get the RTC going
  Wire.begin(i2cSDA,i2cSCL);
  #ifdef LCD_DISPLAY
  char lcd_buf[21];
  lcd.init();
  lcd.backlight();  // backlight on
  lcd.print("CoopTroller   ");
  lcd.print(CoopTrollerVersion);
  lcd.setCursor(0,1);
  lcd.print("2016 - Pablo Sanchez");
  lcd.setCursor(0,2);
  DateTime now = DateTime(__DATE__,__TIME__);
  snprintf(lcd_buf,21,"Comp:%02i/%02i/%2i %02i:%02i",now.month(),now.day(),(now.year()-2000),now.hour(),now.minute());
  lcd.print(lcd_buf);
  lcd.setCursor(0,3);
  lcd.print("");
  /*
  lcd.print(now.day(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.year(), DEC);
  lcd.print(' ');
  if (now.hour()<10)
    lcd.print('0');
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute()<10)
    lcd.print('0');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  if (now.second()<10)
    lcd.print('0');
  lcd.print(now.second(), DEC);
  */
  //lcd.init();
  #endif

  // Set the outputs
  if (! RTC.begin()) {
    #ifdef LCD_DISPLAY
    lcd.setCursor(0,3);
    lcd.print("RTC not found! Halting.");
    #endif
    Serial.println("Couldn't find RTC");
    while (1);
  }
  #ifdef LCD_DISPLAY
  delay(7000);
  #endif
  /* RTC_DS1307
  if (! RTC.isrunning() ) {
    //clockmode = NOT_SET;
    if (Debugging) {
       Serial.println("RTC is not running");
    }
    RTC.adjust(DateTime(__DATE__, __TIME__));  // try kick starting the clock with the compile time
  }
  */
  if (Debugging) {
    now = RTC.now();
    Serial.printf("RTC time is: %0d:%0d\n",now.hour(),now.minute());
  }

  if (Debugging) {
    Serial.println("ARDUINO: Setup WIFI");
  }

  WiFiConfig(0);

  //WiFi.begin(wHost, wPass);

  //int wifitry = 60; // try for 30 seconds then move on
  //while (WiFi.status() != WL_CONNECTED &&  wifitry >0 ) {
  //  delay(100);
  //  wifitry--;
  //  if (Debugging) {
  //     Serial.print(".");
  //  }
  //}

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi connected");
       Serial.println("IP address: ");
       Serial.println(WiFi.localIP());
       Serial.println("MQTT info: ");
       Serial.printf("Status Topic: %s\n",pStatus);
       Serial.printf("Server/port: %s/%s\n",mqtt_server, mqtt_port);  // client_id, port
       Serial.printf("Client ID: %s\n",mClientID);
    }
  } else {
    wifiConnected = false;
    /*
    #ifdef LCD_DISPLAY
    lcd.clear();
    lcd.print("No WiFi");
    #endif
    */
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi UNconnected");
    }
  }

  if (Debugging) {
    Serial.println("ARDUINO: Setup MQTT client");
    Serial.printf("Client|port: %s|%s\n",mqtt_server,mqtt_port);
  }

  mqtt.setCallback(&mqttData);
  mqtt.setServer(mqtt_server, stringToNumber(mqtt_port));  // client_id, port
  mqtt.connect(mClientID);

  delay(100);
  if ( !mqtt.connected() ) {
     mqtt.connect(mClientID);
     delay(100);
  }

  if ( !mqtt.connected() ) {
    if (Debugging) {
      Serial.println("ARDUINO: Failed to setup MQTT");
    }
  }

  if (Debugging) {
    Serial.print("MQTT Status: ");
    Serial.println(mqtt.state());
  }

  //WiFi.onEvent(wifiCb);

  mqttSubscribe();

  // Findout door status
  debounceTop.update();
  debounceBot.update();

  doorTopState    = debounceTop.read();
  doorBottomState = debounceBot.read();

  switch (doorTopState + doorBottomState) {
    case 0:
       doorState = "broken";
       if (Debugging) {
           Serial.println("Door broken. Opened and closed simultaneously. Halting");
       }
       #ifdef LCD_DISPLAY
       //lcd.setCursor(0,2);
       //lcd.print("Door sensors Halting");
       lcdUpdate();
       #endif
       if (wifiConnected) {
          mqtt.publish(pStatus,"door|insane");
       }
       // Goto to the corner and stay there
       while(1){
        yield();
       }
       break;
    case 1:
       if(doorTopState==0){
          doorState="open";
          if (wifiConnected) {
             mqtt.publish(pStatus, "door|open");
          }
       } else {
          doorState="closed";
          if (wifiConnected) {
             mqtt.publish(pStatus, "door|closed");
          }
       }
       break;
    case 2:
       doorState = "unknown";
       if (wifiConnected) {
          mqtt.publish(pStatus, "door|unknown");
       }
       break;
  }

  if (Debugging) {
    Serial.print("Initial door state: ");
    Serial.println(doorState);
  }
  // Port defaults to 8266 for OTA
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Basic OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * Main program loop
 */
void loop() {
  //if (Debugging) {
  //  Serial.println("Loop start");
  //}
  ArduinoOTA.handle(); // Check for OTA activity

  // See if user has requested a new WiFi config
  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    //reset settings to allow a new WiFi connection
    //WiFi.removeEvent(wifiCb);
    WiFiConfig(1);
    //WiFi.onEvent(wifiCb);
  }

  // Update the LCD screen
  #ifdef LCD_DISPLAY
  //if (Debugging) {
  //  Serial.println("LCD Update");
  //}
  lcdUpdate();
  #endif
  //if (Debugging) {
  //  Serial.println("MQTT Update");
  //}

  // Reconnect MQTT if needed
  if ( !mqtt.connected() ) {
    //if (Debugging) {
    //  Serial.println("MQTT Re-connect");
    //}
    mqtt.connect(mClientID);
    if ( mqtt.connected() )  {
      mqttSubscribe();
    }
  } else {
    mqtt.loop();  // MQTT queue flush
  }

  debounceTop.update();
  doorTopState = debounceTop.read();
  debounceBot.update();
  doorBottomState = debounceBot.read();

  // Read new data from sensors
  //if (Debugging) {
  //  Serial.println("Read Sensors");
  //}
  readSensors();

  if (remoteLockStart == 0 || (unsigned long)(millis() - remoteLockStart) > remoteOverride) {
    // Respond to sensor data
    handleSensorReadings();
  }

  // Move the door as needed
  //if (Debugging) {
  //  Serial.println("Door Move");
  //}
  doorMove();
}
