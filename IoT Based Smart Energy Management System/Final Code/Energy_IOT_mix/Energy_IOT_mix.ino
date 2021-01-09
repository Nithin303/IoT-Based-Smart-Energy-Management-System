
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "ACS712.h"
#include <DS3231.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define MainR           D0
#define Relay1          D1   //led 1
#define Relay2          D2   //led 2 
#define Relay3          D3   //Fan 
#define Change_over     D8    
#define current_sensor  A0

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "OnePlus"
#define WLAN_PASS       "123456789"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "aslahahamed"
#define AIO_KEY         "aio_dSew79HwnRrtfWTGgsVLKvoZM21K"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

SoftwareSerial s(D6,D5);
ACS712 sensor(ACS712_20A, current_sensor);
RTClib RTC;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/


// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe Main = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/MainR");
Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay1");
Adafruit_MQTT_Subscribe Light2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay2");
Adafruit_MQTT_Subscribe Fan = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay3");
Adafruit_MQTT_Subscribe coSwitch = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Change_over");
Adafruit_MQTT_Publish mVolt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/BT_level");
Adafruit_MQTT_Publish BCO = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Battery");//batry charging Option
Adafruit_MQTT_Publish BL = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/test");
Adafruit_MQTT_Publish Power = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/watt");
Adafruit_MQTT_Publish current = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Current");
Adafruit_MQTT_Publish co_Switch = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Change_over");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
float get_sensor(char);
void check_Current();

float bv,mv,u;
int unit;
int hou;

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  Wire.begin(D7,3);                           //(SDA, SCL);rx pin 3
  
  pinMode(MainR,OUTPUT);
  pinMode(Relay1,OUTPUT);
  pinMode(Relay2,OUTPUT);
  pinMode(Relay3,OUTPUT);
  pinMode(Change_over,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(Relay1,HIGH);
  digitalWrite(Relay2,HIGH);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Main);
  mqtt.subscribe(&Light1);
  mqtt.subscribe(&Light2);
  mqtt.subscribe(&Fan);
  mqtt.subscribe(&coSwitch);
  
  float zero_point = sensor.calibrate();
  sensor.setZeroPoint(zero_point);
}


void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10000))) {
    if (subscription == &Main) {
      Serial.print(F("Got: "));
      Serial.println((char *)Main.lastread);
      int Main_State = atoi((char *)Main.lastread);
      digitalWrite(MainR,!Main_State);
    }
    if (subscription == &Light1) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light1.lastread);
      int Light1_State = atoi((char *)Light1.lastread);
      digitalWrite(Relay1,!Light1_State);
    }
    if (subscription == &Light2) {
      Serial.print(F("Got: "));
      Serial.println((char *)Light2.lastread);
      int Light2_State = atoi((char *)Light2.lastread);
      digitalWrite(Relay2,!Light2_State);
    }
    if (subscription == &Fan) {
      Serial.print(F("Got: "));
      Serial.println((char *)Fan.lastread);
      int Fan_State = atoi((char *)Fan.lastread);
      digitalWrite(Relay3,Fan_State);
    }
    if (subscription == &coSwitch) {
      Serial.print(F("Got: "));
      Serial.println((char *)coSwitch.lastread);
      String coSwitch_S = ((char *)coSwitch.lastread);
      if(coSwitch_S == "Solar"){
        digitalWrite(Change_over,HIGH);
      }
      else{
        digitalWrite(Change_over,LOW);
      }
    }
  } 
  check_Current();
  bv=get_sensor('b');
  checkBatteryLevel(bv);
  mv=get_sensor('m');
  Serial.print("Main Voltage :");
  Serial.print(mv);
  if (! mVolt.publish(mv)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F(" OK!"));
  }
  u=get_sensor('p');
  unit=u*1;
  Serial.print("Kwh : ");
  Serial.print(unit);
  if (!Power.publish(unit)) {
    Serial.println(F(" Failed"));} 
  else {
    Serial.println(F(" OK!"));}

  DateTime now = RTC.now();
  hou=now.hour();
  //Serial.print("Time : ");Serial.print(hou);Serial.print(":");Serial.println(now.minute(), DEC);
  if(hou==18 && bv>=11.6)
  {
    digitalWrite(Change_over,HIGH);
    Serial.print("Connected to Solar");
    if (!co_Switch.publish("Solar")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.println(F(" OK!"));}
  }
  if(hou==22 || bv<=11.6)
  {
    digitalWrite(Change_over,LOW);
    Serial.print("Connected to Utility");
    if (!co_Switch.publish("Utility")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.println(F(" OK!"));}
  }
  
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    Serial.println(inChar);
    switch(inChar){
      case '1':
      if (!BCO.publish(1)) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.println(F(" OK!"));}
      break;
      case '0':
      if (!BCO.publish(0)) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.println(F(" OK!"));}
      break;
    }
  }
  Serial.println("");
  delay(1500);
  }

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}


/***************************************************************************************
                                    get sensor value
 ****************************************************************************************/
 float get_sensor(char opt) {
  float data;
  s.write(opt);
  delay(10);
  while(s.available()==0);
  data=s.read();
 // Serial.println(data);
  return data;
}


/*****************************************************************************************/

void checkBatteryLevel(float val)
{
  int Vp;
  Serial.print("Battey Voltage : ");Serial.println(val);
  if(val<=13.5 && val>=11.5){
  Vp = ((val-11.5)/2)*100;
  Serial.print("Battey Level : ");Serial.print(Vp);Serial.println(" %");}
  else{
    Serial.print("Battey Level : ");Serial.println("0 %");}
  if(val<=13.7 && val>13)
  {
    if (!BL.publish("battery-full")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.print("Battery-full");
      Serial.println(F(" OK!"));}    
  }
  else if(val<=13 && val>12.5)
  {
    if (!BL.publish("battery-3")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.print("Battery-3");
      Serial.println(F(" OK!"));}
  }
  else if(val<=12.5 && val>12)
  {
    if (!BL.publish("battery-2")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.print("Battery-2");
      Serial.println(F(" OK!"));}
  }
  else if(val<=12 && val>11.6)
  {
    if (!BL.publish("battery-1")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.print("Battery-1");
      Serial.println(F(" OK!"));}
      
  }
  else
  {
    if (!BL.publish("battery-0")) {
      Serial.println(F(" Failed"));} 
      else {
      Serial.print("Battery-0");
      Serial.println(F(" OK!"));}  
  }
}


/***********************************************************************************/

void check_Current()
{
  float I = sensor.getCurrentAC();
  Serial.print("Current : ");
  I-=.9;                     
  if(I<0){
    I*=-1;
  }
  Serial.print(I);
  if (! current.publish(I)) {
    Serial.println(F(" Failed"));
  } else {
    Serial.println(F(" OK!"));
  }
}

 
 
