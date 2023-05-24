#include <Arduino.h>
/*
Design of an IoT system for monitoring chemical
and physical properties of soil (moisture,
temperature, pH), precipitation, pest intrusion
for pineapple and tomato crops

Developed by Jonathan Gheysens & Fidele Houeto 

Example for DRAMCO-UNO

Connection wires of SMT-01 cable to DRAMCO UNO:
Yellow DS  - to Pin 5 (R2 4.7k - 2.0k to +5V, depending on length of the cable)
Green 2N   - to Pin 4 (via R1 (10k - 2.0k, depending on length of the cable)
Red        - to +5V
Black      - to GND

Connection wires of FC-37 rain sensor:
Blue  - to Pin A0
Red   - to +5V
Black - to GND

Connection wires of HC-SR04 sensor:
Pink - to pin 10 (trig)
Cyan - to pin 9 (echo)
Red  - to +5V
Black- to GND

Connection wires of MA40S4S ultrasonic transducer:
Purple  - to pin 8
Black- to GND
*/

//definitions and initialisations for DRAMCO-UNO board
#include <Dramco-UNO.h>

LoraParam DevEUI = "70B3D57ED005DD24";
LoraParam AppKey = "24444A42B9CCA5F41CAC4D9C3C0048F6";


//definitions and initialisations for SMT-01 sensor
#include <OneWire.h>

#define DARK  LOW
#define LIGHT HIGH
#define ON    HIGH
#define OFF   LOW

OneWire  ds(5);  // 1-Wire to Pin 5: sensor DS18B20 connected

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius;
int m_err;

int pin_Led = 13; //BUILTIN-LED
int pin_Heater = 4; //pin heating component: bipolar transistor 2N2222A        

int DS_found = 0;                   
float Time_Heat_Dissipation, Soil_Moisture, Soil_Temperature;
unsigned long Heating_Time = 30000; //ms
int j, k;

void DS18B20_init(void);
void DS18B20_measure(void);
void Measure_SMT (void);

unsigned long mtime;                  
unsigned long set_mtime;    


//definitions and initialisations for FC-37 sensor
int rainPin = A0;
int thresholdValue = 500; //adjust this value for your own desing

//definitions and initialisations for HC-SR04 sensor
int trigPin = 10;
int echoPin = 9;
long duration;
float distance;

//definitions and initialisations for MA40S4S ultrasonic transducer
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int ultrasonicPin = 8;

//setup
void setup()
{
   pinMode(pin_Led,OUTPUT);
   pinMode(pin_Heater, OUTPUT);

   pinMode(rainPin, INPUT);

   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
  
   digitalWrite(pin_Led,DARK);
   digitalWrite(pin_Heater, LOW);

//LORAWAN CONNECTIVITY
   DramcoUno.begin(DevEUI, AppKey);
   
// UART communication setup: easy to read out with serial monitor  
   Serial.begin(9600);
   delay(10);
   Serial.println("");

   Serial.println("Initialization of DS18B20 ... ");

   DS_found = 0;
   DS18B20_init();
   if (DS_found == 1){
       digitalWrite(pin_Led, LIGHT); 
       Serial.println("Initialization is Ok");
       delay(1000);
       digitalWrite(pin_Led,DARK);
      }

   set_mtime = 1000;
   mtime = millis();
 
}//setup


void loop()
{ //rain sensor
  int rainValue = analogRead(rainPin);
  Serial.print(rainValue);
  if(rainValue < thresholdValue){
    Serial.println(" - It's raining");
  }
  else {
    Serial.println(" - It's not raining");
  }
  DramcoUno.sendRain(rainValue);

  // HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  DramcoUno.sendUltrasonic(distance);
  
  //ultasonic wave
  if (distance<MAX_DISTANCE){//pest intrusion detected: reflection of ultrasonic wave by pest
      tone(ultrasonicPin, 65000, 1000);//ultrasonic wave at 65kHz for 1 second
  }

 if (millis() - mtime > set_mtime) { 
  
  digitalWrite(pin_Led, LIGHT);
  
// Measurement

  if(DS_found == 1){
  
  Serial.println("Start measurement");
  Measure_SMT();

// Converting the Time of Heat Dissipation to Soil Moisture, %

// as example
  float Sensor_Dry = 250.0; //Time of Heat Dissipation for Dry Sensor
  float Sensor_Wet = 35.0;  //Time of Heat Dissipation for Wet Sensor
  
  Soil_Moisture = map(Time_Heat_Dissipation, Sensor_Dry, Sensor_Wet, 0.0, 100.0);
  if (Soil_Moisture < 0.0) Soil_Moisture = 0.0;
  if (Soil_Moisture > 100.0) Soil_Moisture = 100.0;

  Serial.print("Soil Moisture = ");
  Serial.print(Soil_Moisture);
  Serial.println(", %");
  DramcoUno.sendSoilMoisture(Soil_Moisture);
  Serial.print("Temperature of Soil = ");
  Serial.print(Soil_Temperature);
  Serial.println(", oC");
  DramcoUno.sendSoilTemperature(Soil_Temperature);
  }
  else{
   Serial.println("Next try to Initialization of DS18B20 ... ");
   DS_found = 0;
   DS18B20_init();
   if (DS_found == 1){
       digitalWrite(pin_Led, LIGHT); 
       Serial.println("Initialization is Ok");
       delay(1000);
       digitalWrite(pin_Led,DARK);
      }
    
    }
  
  set_mtime = 420000; // recommended pause between measurements, ms (7 minutes)
  mtime = millis();

  Serial.println("Waiting for the next measurement ...");
  digitalWrite(pin_Led,DARK);
  
  }// if mtime

  delay(10);
  
}// loop


void DS18B20_init(void){ //--------------------------------------------------------------

if ( !ds.search(addr)) {
    DS_found = 0;
    Serial.println("Sensor not found.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }//if

Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      DS_found = 0;
      return;
  }
  Serial.println();

// the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      DS_found = 1;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }//switch

}//DS18B20_init 

void DS18B20_measure(void) {//------------------------------------------------------

  m_err = 0;
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);    // start conversion, with parasite power on at the end
  
  delay(1000);          // time need for conversion
    
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);       // Read Scratchpad

  for ( i = 0; i < 12; i++) {           // we need 12 bytes resolution
    data[i] = ds.read();
  }

 if (OneWire::crc8(data, 8) != data[8]) {
      Serial.println("CRC is not valid!");
      m_err = 1;
  }

  
  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  celsius = (float)raw / 16.0;
  
 }//DS18B20_measure
  
void Measure_SMT ()
   {
    float t_current, tj;
    int m_cycle, j, m;
    unsigned long dtime;              

    Time_Heat_Dissipation = 0.0;
    j = 0;
    m = 0;
    tj = 0.0;

    Serial.println("Temperature of Soil measurement ...");
    for (m_cycle = 0; m_cycle<10; m_cycle++)
        { 
          DS18B20_measure();
          if (m_err == 0)
             {
              tj = tj + celsius; 
              j++; 
              Serial.println(celsius);
             }//if
          else {m++;}   
                    
         }//for

    if (j > 0 && m < 7)  { Soil_Temperature = tj/j; }
    else {Soil_Temperature = -21.0;} 
       
    if (Soil_Temperature > 0.0) {

    Serial.println("Heating ..."); 
    digitalWrite(pin_Heater, HIGH);
    delay(Heating_Time);                    // Time of heating, ms
    digitalWrite(pin_Heater, LOW);
    
    Serial.println("Heat dissipation ..."); 
    dtime = millis(); // start time of Heat dissipation

    t_current = (Soil_Temperature + 5.0); 
    DS18B20_measure();
    if (m_err == 0) {t_current = celsius;}
    
    m_cycle = 0;
        
    while (t_current > (Soil_Temperature + 1.0) && m_cycle < 250)
        { 
         DS18B20_measure();
         if (m_err == 0) {t_current = celsius;}
         Serial.println(t_current );
         m_cycle++;
        }//while

    Time_Heat_Dissipation = (millis() - dtime)/1000.0;
    Serial.print("Time of Heat Dissipation = ");
    Serial.print(Time_Heat_Dissipation);
    Serial.println(", seconds");
     
   }//Soil_Temperature > 0
   
}// Measure_SMT