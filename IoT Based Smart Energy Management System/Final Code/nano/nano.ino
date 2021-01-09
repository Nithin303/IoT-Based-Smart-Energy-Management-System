//Arduino code
#include <EEPROM.h>
#include <SoftwareSerial.h>
SoftwareSerial s(5,6);

char cmd;
int def=0;
float mv,bv;
int mVolt,bVolt;
int pulse_pin = 12;
float unit,w;
int call=32;                   //3200
int pulse;
int addr = 0;

float check_pulse();
 
void setup() {
  s.begin(9600);
  Serial.begin(9600);
  pinMode(pulse_pin, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //EEPROM.write(addr, 5);
  //EEPROM.write(4, 0);
  unit = EEPROM.read(addr);
  pulse = EEPROM.read(4);
}
 
void loop() {
while(s.available() > 0)//while(s.available() > 0)
{
  cmd=s.read();
  Serial.println(cmd);
  switch(cmd)
  {
    case 'm':
    mVolt= analogRead(A1);
    mv = map(mVolt, 0, 1024, 0, 36000);// for converting into 230 range
    mv/=100;
    s.write(mv);
    Serial.println(mv);
    break;
    case 'b':
    bVolt= analogRead(A2);
    bv = map(bVolt, 0, 1024, 0, 2485);
    bv/=100;
    s.write(bv);
    Serial.println(bv);
    break;
    case 'p':
    w=check_pulse();
    Serial.println(w);
    s.write(w);
    break;
    default:
    s.write(def);
    break;
  }
}
w=check_pulse();
}


/***************************************************************/

float check_pulse() {
  int pulseState = digitalRead(pulse_pin);
  if(pulseState == LOW)
  {
    pulse++;
    EEPROM.write(4, pulse);
    if(pulse==call)
    {
      pulse=0;
      unit++;
      EEPROM.write(4, pulse);
      EEPROM.write(addr, unit);
      Serial.println(unit);
    }
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    Serial.print("Pulse Received : ");
    Serial.println(pulse);
    Serial.println(unit);
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW   
    return unit;
  }
   delay(1);
   return unit;
}
