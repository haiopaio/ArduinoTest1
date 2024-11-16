/* ###################################
Mit Zeitverzögerungen Zeile 42 und 222, activeOn in Zeile 50, 102
Das Poti stellt normalerweise den Sollwert ein. 
und optional manuelle Steuerung des Servos durch das Poti in Zeile 115 
#######################################*/


/* Der Teil für die PT-100 Messung basiert auf der Library "Adafruit_MAX31865_library" und
* dem Beipiel "max31865.ino"
*
* Der Teil für die OLED Anzeige basiert auf der Library "SSD1306Ascii" und
* dem Beispiel "Wire128x64.ino"
*
* Der Teil für die Servo Ansteuerung basiert auf der internen Library "servo.h" und
* dem Tutorial https://docs.arduino.cc/learn/electronics/servo-motors/
*/

#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Servo.h>
#include <elapsedMillis.h>

#define RREF      430.0 // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RNOMINAL  100.0 // The 'nominal' 0-degrees-C resistance of the sensor  

#define I2C_ADDRESS 0x3C // for OLED 0X3C+SA0 - 0x3C or 0x3D
#define RST_PIN -1 // Define proper RST_PIN for OLED if required.

//erzeugt 2 Thermometer Objekte und definiert die verwendeten Pins
Adafruit_MAX31865 thermo_A = Adafruit_MAX31865(10, 11, 12, 13);// Use software SPI: CS, DI, DO, CLK // Define pins for dual MAX31865
Adafruit_MAX31865 thermo_B = Adafruit_MAX31865(9, 11, 12, 13);

SSD1306AsciiWire oled; // erzeugt ein SSD1306 Objekt "oled"
Servo myservo;    // erzeugt ein Servomotor-Objekt "myservo" // maximal können acht Servomotor-Objekte erzeugt werden

// Ventil Max und Min Stellungen
int vent_max = 0;// Servo voll CCW, von hinten gesehen - Ventil voll offen
int vent_min = 160;// Servo voll CW, von hinten gesehen - Ventil nur noch leicht offen
int vent_set = vent_min - 0; // Variable, die die Servoposition (Winkel 180 bis 0 Grad) speichert
bool activeOn {}; // wird true wenn KW Ausgang erstmals deutlich höher wird als KW Eingang

elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs
// delay in milliseconds
unsigned int interval1 = 1000; // Zeitverzögerung in mSec pro Servo Schritt Ventil auf
unsigned int interval2 = 2000; // Zeitverzögerung in mSec pro Servo Schritt Ventil zu


void setup() 
{
  activeOn = false;

  Serial.begin(115000); // Ausgabe über seriellen Monitor
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  thermo_A.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo_B.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  Wire.begin();
  Wire.setClock(400000L);
  oled.setFont(Adafruit5x7);
   #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN < 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif //

  uint32_t m = micros();
  oled.clear();
  oled.println("Hello world!");
  oled.println("A long line may be truncated");
  oled.println();
  oled.set2X();
  oled.println("2X demo");
  oled.set1X();
  oled.print("\nmicros: ");
  oled.print(micros() - m);
  delay(1000);
  oled.clear();
  
  myservo.attach(7);  // an welchem Pin ist der Servomotor angeschlossen
  myservo.write(vent_min-15); // Grundstellung, leicht geöffnet
  delay(100);

  pinMode(13, OUTPUT);
  digitalWrite(13,1);
}

void loop() 
{
  float degC_A {}; // KW Ausgang
  float degC_B {}; // KW Eingang
  float delta_T {}; // KwAus minus KwEin
  float offset_A = -0.2; // Korrekturwert A für die Kalibrierung (pos. Wert erhöht die Anzeige)
  float offset_B = -0.2; // Korrekturwert B für die Kalibrierung (pos. Wert erhöht die Anzeige)
  long int error_T {};

  degC_A = readSensorA(offset_A);// Sensor A Temperatur speichern (KW Ausgang) mit Kalibrieroffset
  printFaultA();
  degC_B = readSensorB(offset_B);// Sensor B Temperatur speichern (KW Eingang) mit Kalibrieroffset
  printFaultA();

  delta_T = degC_A - degC_B; // KwAus minus KwEin

  if (activeOn == false && delta_T > 8){  // öffnet das Ventil einmalig, wenn erstmals KW Aus >> KW Ein
    vent_set = 115;
    myservo.write(vent_set); delay(500);
    activeOn = true;    
  }
   
  // Poti für Sollwert-Einstellung
  long int pot1Value = analogRead(A0);   // read the input on analog pin 0
  float sollWert = map(pot1Value, 0, 1023, 0, 60); // speichere als Sollwert für delta_T vom Poti1


  vent_set = map(pot1Value, 0, 1023, 175, 60); // Nur für Test - Direktsteuerung des Servos durch Poti
  myservo.write(vent_set);// Servo bewegt das Ventil
  delay(15);

/*########Für manuelle Servopositionierung auskommentieren
  error_T = delta_T - sollWert; // Fehlersignal für die Regelung der Servoposition
  myservo.write(servo_set (error_T));// Servo bewegt das Ventil
  delay(15);
##########*/
  
  
  printSerial (sollWert, delta_T, error_T, vent_set);

  printOled (degC_A, degC_B, delta_T, sollWert, error_T, vent_set);
 
 


  
  
  delay(50);

} // Ende loop ********************************************

//################Funktionen##########################

// Read Sensor A KW Ausgang mit Kalibrieroffset
float readSensorA(float offset_A){
  return thermo_A.temperature(RNOMINAL, RREF)+ offset_A;
}

// Read Sensor B KW Eingang mit Kalibrieroffset
float readSensorB(float offset_B){
  return thermo_B.temperature(RNOMINAL, RREF)+ offset_B;
}

// Check and print any faults Sensor A
void printFaultA(){
    uint8_t fault_A = thermo_A.readFault();
    if (fault_A) 
    {
    Serial.print("Fault 0x"); Serial.println(fault_A, HEX);
    if (fault_A & MAX31865_FAULT_HIGHTHRESH) 
      {
      Serial.println("RTD High Threshold"); 
      }
    if (fault_A & MAX31865_FAULT_LOWTHRESH) 
      {
      Serial.println("RTD Low Threshold"); 
      }
    if (fault_A & MAX31865_FAULT_REFINLOW) 
      {
      Serial.println("REFIN- > 0.85 x Bias"); 
      }
    if (fault_A & MAX31865_FAULT_REFINHIGH) 
      {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
    if (fault_A & MAX31865_FAULT_RTDINLOW) 
      {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
    if (fault_A & MAX31865_FAULT_OVUV) 
      {
      Serial.println("Under/Over voltage"); 
      }
    thermo_A.clearFault();
    }
}

// Check and print any faults Sensor B
void printFaultB(){
    uint8_t fault_B = thermo_B.readFault();
    if (fault_B) 
    {
    Serial.print("Fault 0x"); Serial.println(fault_B, HEX);
    if (fault_B & MAX31865_FAULT_HIGHTHRESH) 
    {
      Serial.println("RTD High Threshold"); 
    }
    if (fault_B & MAX31865_FAULT_LOWTHRESH) 
    {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault_B & MAX31865_FAULT_REFINLOW) 
    {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault_B & MAX31865_FAULT_REFINHIGH) 
    {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault_B & MAX31865_FAULT_RTDINLOW) 
    {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault_B & MAX31865_FAULT_OVUV) 
    {
      Serial.println("Under/Over voltage"); 
    }
    thermo_B.clearFault();
    }
}

// Berechne Servo Stellung aus dem Verhalten von error_T 
int servo_set (long int error_T){
  
   if (error_T >= 20)
    {
      if (timeElapsed > interval1){
        vent_set -=10; 
        vent_set = constrain (vent_set, vent_max, vent_min);//Reduzierung vent_set öffnet das Ventil
        timeElapsed = 0; // reset the counter to 0 so the counting starts over...
      }
    }
  else if (error_T  >= 5 && error_T  < 20)
    {
      if (timeElapsed > interval1){
        vent_set -=5;
        vent_set = constrain (vent_set, vent_max, vent_min);
        timeElapsed = 0; // reset the counter to 0 so the counting starts over...
      }
    }
  

  else if (error_T  <= -5 && error_T > -20)
    {
      if (timeElapsed > interval2){
        vent_set +=5;
        vent_set = constrain (vent_set, vent_max, vent_min);//Erhöhung vent_set schließt das Ventil
        timeElapsed = 0; // reset the counter to 0 so the counting starts over...
      }
    }
  else if (error_T  <= -20)
    {
      if (timeElapsed > interval2){
        vent_set +=10;
        vent_set = constrain (vent_set, vent_max, vent_min);
        timeElapsed = 0; // reset the counter to 0 so the counting starts over...
      }
    }
  
 
  return vent_set;
}

// Drucke 4 Werte in einer Zeile über Serial Monitor
void printSerial (float sollWert, float delta_T, long error_T, int vent_set){
  Serial.print("Sollwert: " ); Serial.print(sollWert); 
  Serial.print("     ");  
  Serial.print("   Delta_T: " ); Serial.print(delta_T);  
  Serial.print("     ");  
  Serial.print("   error_T: " ); Serial.print(error_T);  
  Serial.print("     ");  
  Serial.print("   vent_set: " );  Serial.println(vent_set);
}

// Drucke 6 Werte auf das OLED
void printOled (float degC_A, float degC_B, float delta_T, float sollWert, long error_T, int vent_set){
  
  oled.setCursor(0,0);
 
  oled.set2X(); oled.print("KwA: "); oled.println(degC_A); // KW Ausgang
  
  oled.set1X(); oled.print("KwE: ");  oled.println(degC_B); // KW Eingang
  
  oled.set2X(); oled.print("dT:"); oled.print(' ');oled.println(delta_T);

  oled.set1X(); oled.print("sW:"); oled.print(' '); oled.print(sollWert);
  oled.print(' '); oled.print("er_T: "); oled.println(float(error_T)); // Sollwert - delta_T

  oled.set1X(); oled.print("v_set:   "); 
  oled.set2X();oled.print (float(vent_set));// Ausgabe Ventilstellung, 180 = geschlossen, 0 = offen
}