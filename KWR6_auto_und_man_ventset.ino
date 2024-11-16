/* ###################################
Mit Zeitverzögerungen Zeile 42 und 222, activeOn in Zeile 50, 102
Das Poti stellt normalerweise den Sollwert ein. 
und optional manuelle Steuerung des Servos durch das Poti in Zeile 115 
#######################################*/


/* ####################################
Die PT-100 Messung basiert auf "Adafruit_MAX31865_library" und dem Beipiel "max31865.ino".
Die OLED Anzeige basiert auf der Library "SSD1306Ascii" und dem Beispiel "Wire128x64.ino".
Die Servo Ansteuerung basiert auf der internen Library "servo.h" und dem Tutorial
https://docs.arduino.cc/learn/electronics/servo-motors/
Die Zeitverzögerung für die Servoschritte basiert auf der Library "elapsedMillis.h".
########################################*/

#include <Adafruit_MAX31865.h>  // für PT-100 Messung
#include <Wire.h>               // für OLED I2C
#include "SSD1306Ascii.h"       // für OLED I2C
#include "SSD1306AsciiWire.h"   // für OLED I2C
#include <Servo.h>              // für Servo Ansteuerung
#include <elapsedMillis.h>      // für Die Zeitverzögerung der Servo Schritte, ohne den Loop mit delay() aufzuhslten

#define RREF 430.0      // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RNOMINAL 100.0  // The 'nominal' 0-degrees-C resistance of the sensor

#define I2C_ADDRESS 0x3C  // for OLED 0X3C+SA0 - 0x3C or 0x3D
#define RST_PIN -1        // Use internal Arduino Reset for OLED.
//#### Objekte erzeugen####
//erzeugt 2 Thermometer Objekte und definiert die verwendeten Pins ffür das SPI Interface
Adafruit_MAX31865 thermo_A = Adafruit_MAX31865(10, 11, 12, 13);  // Use software SPI: CS, DI, DO, CLK // Define pins for dual MAX31865
Adafruit_MAX31865 thermo_B = Adafruit_MAX31865(9, 11, 12, 13);
// erzeugt ein SSD1306 Objekt "oled"
SSD1306AsciiWire oled;
// erzeugt ein Servomotor-Objekt "myservo" // maximal können acht Servomotor-Objekte erzeugt werden
Servo myservo;
// Timer Objekte (global, sonst wird der Timer bei jedem loop neu gestartet)
elapsedMillis timeElapsed1;  //Timer für "Ventil öffnen"
elapsedMillis timeElapsed2;  //Timer für "Ventil schließen"
elapsedMillis timeElapsed3;  //Timer für langsameren Serial Print

//######## globale Variablen######
// Ventil Steuerung
int vent_max = 0;             // Servo voll CCW, von hinten gesehen - Ventil voll offen
int vent_min = 165;           // Servo voll CW, von hinten gesehen - Ventil nur noch leicht offen
int vent_set = vent_min - 0;  // Variable, die die Servoposition (Winkel 180 bis 0 Grad) speichert
bool activeOn = false;        // wird true wenn KW Ausgang erstmals deutlich höher wird als KW Eingang

// Grenzwerte für Timer (global)
unsigned int interval1 = 1500;  // Zeitverzögerung in mSec pro Servo Schritt Ventil auf
unsigned int interval2 = 3000;  // Zeitverzögerung in mSec pro Servo Schritt Ventil zu
unsigned int interval3 = 100;   // Zeitverzögerung in mSec pro Serial Data Output


void setup() {
  activeOn = false;  // Servo-Steuerung zu Beginn deaktiviert

  Serial.begin(115000);  // für Ausgabe über seriellen Monitor aktivieren

  thermo_A.begin(MAX31865_4WIRE);  // PT-100 4-Wire aktivieren
  thermo_B.begin(MAX31865_4WIRE);  // PT-100 4-Wire aktivieren

  Wire.begin();  // I2C Bus aktivieren
  Wire.setClock(400000L);
  oled.setFont(Adafruit5x7);
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);

  myservo.attach(7);             // an welchem Pin ist der Servomotor angeschlossen
  myservo.write(vent_min - 35);  // Ventil leicht öffnen um Durchfluss sicherzustellen
  delay(1000);
}

void loop() {
  float degC_A{};         // KW Ausgang
  float degC_B{};         // KW Eingang
  int delta_T{};          // KwAus minus KwEin
  float offset_A = -0.2;  // Korrekturwert A für die Kalibrierung (pos. Wert erhöht die Anzeige)
  float offset_B = -0.2;  // Korrekturwert B für die Kalibrierung (pos. Wert erhöht die Anzeige)
  int error_T{};

  degC_A = readSensorA(offset_A);  // Sensor A Temperatur speichern (KW Ausgang) mit Kalibrieroffset
  printFaultA();
  degC_B = readSensorB(offset_B);  // Sensor B Temperatur speichern (KW Eingang) mit Kalibrieroffset
  printFaultA();

  delta_T = degC_A - degC_B;  // KwAus minus KwEin, Ergebnis als Integer Value

  if (activeOn == false && delta_T > 4) {  // öffnet das Ventil einmalig, wenn erstmals KW Aus >> KW Ein
    vent_set = 115;                        // ca. 50% Durchfluss
    myservo.write(vent_set);
    delay(500);
    activeOn = true;  // und schaltet die Regelung auf aktiv
  }


  // Poti für Sollwert-Einstellung
  long int pot1Value = analogRead(A0);            // read the input on analog pin 0
  int sollWert = map(pot1Value, 0, 1023, 0, 60);  // speichere als Sollwert für delta_T vom Poti1

  /*######### Nur für manuellen Betrieb aktivieren - Für Automatik Betrieb auskommentieren
  vent_set = map(pot1Value, 0, 1023, 175, 60); // Direktsteuerung des Servos durch Poti
  myservo.write(vent_set);// Servo bewegt das Ventil
  delay(15);
  ##########*/

  //  /*######## Nur für Automatik Betrieb aktivieren - Für manuelle Servopositionierung auskommentieren
  error_T = delta_T - sollWert;       // Fehlersignal für die Regelung der Servoposition
  myservo.write(servo_set(error_T));  // Servo bewegt das Ventil ggf. nach Ablauf einer Wartezeit
  delay(15);
  //  ##########*/


  /* Ausgabe nur in bestimmten Zeitabständen um zu verhindern dass sich der Serial Monitor  
    nach längerer Laufzeit aufhängt */
  if (timeElapsed3 > interval3) {
    printSerial(activeOn, sollWert, delta_T, error_T, vent_set);
    timeElapsed3 = 0;
  }


  printOled(degC_A, degC_B, delta_T, sollWert, error_T, vent_set);






  delay(50);

}  // Ende loop ********************************************

//################Funktionen##########################

// Read Sensor A KW Ausgang mit Kalibrieroffset
float readSensorA(float offset_A) {
  return thermo_A.temperature(RNOMINAL, RREF) + offset_A;
}

// Read Sensor B KW Eingang mit Kalibrieroffset
float readSensorB(float offset_B) {
  return thermo_B.temperature(RNOMINAL, RREF) + offset_B;
}

// Check and print any faults Sensor A
void printFaultA() {
  uint8_t fault_A = thermo_A.readFault();
  if (fault_A) {
    Serial.print("Fault 0x");
    Serial.println(fault_A, HEX);
    if (fault_A & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault_A & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault_A & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault_A & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault_A & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault_A & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    thermo_A.clearFault();
  }
}

// Check and print any faults Sensor B
void printFaultB() {
  uint8_t fault_B = thermo_B.readFault();
  if (fault_B) {
    Serial.print("Fault 0x");
    Serial.println(fault_B, HEX);
    if (fault_B & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault_B & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault_B & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault_B & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault_B & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault_B & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    thermo_B.clearFault();
  }
}

// Berechne Servo Stellung aus dem Verhalten von error_T und stelle den Servo nach Ablauf einer Wartezeit
// gibt den vent_set Wert zurück
int servo_set(int error_T) {

  if (error_T >= 50)  // Kühlwasser Ausgang hat Übertemperatur
  {
    vent_set = vent_max;
    vent_set = constrain(vent_set, vent_max, vent_min);  //Öffne das Ventil sofort bis auf vent_max
  } else if (error_T >= 5 && error_T < 50)               // Kühlwasser Ausgang etwas zu warm
  {
    if (timeElapsed1 > interval1) {  // Öffne das Ventil langsam nach Ablauf der Wartezeit oder kehre sofort zurück zu loop
      vent_set -= 5;
      vent_set = constrain(vent_set, vent_max, vent_min);  //Reduzierung vent_set öffnet das Ventil
      timeElapsed1 = 0;                                    // reset the counter to 0 so the counting starts over...
    }
  }


  else if (error_T <= -5 && error_T > -20)  // Kühlwasser Ausgang etwas zu kühl
  {
    if (timeElapsed2 > interval2) {  // Schließe das Ventil langsam nach Ablauf der Wartezeit oder kehre sofort zurück zu loop
      vent_set += 2;
      vent_set = constrain(vent_set, vent_max, vent_min);  //Erhöhung vent_set schließt das Ventil
      timeElapsed2 = 0;                                    // reset the counter to 0 so the counting starts over...
    }
  } else if (error_T <= -20)  // Kühlwasser Ausgang sehr kühl - Vermutlich Heizung aus
  {
    if (timeElapsed2 > interval2) {  //Schließe das Ventil sofort bis auf vent_max
      vent_set = vent_min;
      vent_set = constrain(vent_set, vent_max, vent_min);
      timeElapsed2 = 0;  // reset the counter to 0 so the counting starts over...
    }
  }


  return vent_set;
}

// Drucke 5 Werte in einer Zeile über Serial Monitor
void printSerial(int activeOn, int sollWert, int delta_T, int error_T, int vent_set) {
  Serial.print("activeOn: ");
  Serial.print(activeOn);
  Serial.print("\t");
  Serial.print("Sollwert: ");
  Serial.print(sollWert);
  Serial.print("\t");

  Serial.print("Delta_T: ");
  Serial.print(delta_T);
  Serial.print("\t");

  Serial.print("error_T: ");
  Serial.print(error_T);
  Serial.print("\t");

  Serial.print("vent_set: ");
  Serial.println(vent_set);
}

// Drucke 6 Werte auf das OLED
void printOled(float degC_A, float degC_B, int delta_T, int sollWert, int error_T, int vent_set) {

  oled.setCursor(0, 0);
  oled.set2X();

  oled.print(degC_A, 1);  // KW Ausgang
  oled.print("  ");
  oled.println(degC_B, 1);  // KW Eingang
  // die nachfolgenden Integer Werte werden zur Ausgabe als float übergeben
  // und die Nachkommastellen auf Null gesetzt
  // weil das OLED Integer falsch darstellt
  oled.print("deltaT:");
  oled.print(' ');
  oled.println(float(delta_T), 0);
  oled.print("s:");
  oled.print(float(sollWert), 0);
  oled.print(' ');
  oled.print("r:");
  oled.println(float(error_T), 0);  // Sollwert - delta_T

  if (activeOn == false) {  // drucke Stby: oder On: , abhängig von activeOn Wert
    oled.set2X();
    oled.print("Stby: ");
  } else {
    oled.set2X();
    oled.print("On:   ");
  }
  oled.print(' ');
  oled.print(float(vent_set), 0);  // Ausgabe Ventilstellung, 180 = geschlossen, 0 = offen
}