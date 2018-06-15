/**************************************************************
*
*   Programa per a controlar els dispositius físics
*   de l'escànner 3D.
*
**************************************************************/
#include "TimerOne.h"

// Durada d'un pols de la sortida 'step' per a efectuar un pas.
const int pulse_duration_ms = 2;

//Representació simbòlica dels pins conectats al driver i al sensor òptic (obp).
#define analog_obp A5
const int pin_dir = 13;
const int pin_bot = 12;
const int pin_sleep = 11;
const int pin_reset = 10;
const int pin_m2 = 9;
const int pin_m1 = 8;
const int pin_m0 = 7;
const int pin_enable = 6;
const int pin_fault = 5;
const int pin_obp = 4;

int count;
const int step_count = 25;  // 5.625deg

// Dur a terme un pas
void RotateStep(){
  if (count < step_count) {
    digitalWrite(pin_bot, HIGH);
    delayMicroseconds(pulse_duration_ms);
    digitalWrite(pin_bot, LOW);
    count++;
  }
}

// Configure la velocitat d'escaneig i inicialització del temporitzador
void Configure(const int& period){
    Timer1.stop();
    Timer1.detachInterrupt();

    if (period != 0){
        Serial.write("Scanning at ");
        Serial.write(period);
        Timer1.initialize(period);
        Timer1.attachInterrupt(RotateStep);
    }
}

// Inicialització dels d'entrades i sortides i del node de ROS
void setup(){
  count = 0;
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_bot, OUTPUT);
    pinMode(pin_sleep, OUTPUT);
    pinMode(pin_reset, OUTPUT);
    pinMode(pin_m2, OUTPUT);
    pinMode(pin_m1, OUTPUT);
    pinMode(pin_m0, OUTPUT);
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_fault, INPUT);
    pinMode(pin_obp, OUTPUT);

    digitalWrite(pin_dir, LOW);
    digitalWrite(pin_sleep, HIGH);
    digitalWrite(pin_reset, HIGH);
    digitalWrite(pin_m2, HIGH);
    digitalWrite(pin_m1, HIGH);
    digitalWrite(pin_m0, HIGH);
    digitalWrite(pin_enable, LOW);
    digitalWrite(pin_obp, HIGH);

    Serial.begin(9600);
    Serial.write("Arduino is ready.");
}

// Bucle infinit a l'espera d'interrupcions
void loop(){
    if(Serial.available()>0) {
        int period = Serial.parseInt();
        if (period > 0) {
          Configure(period);
        } else if (period == 0) {
          Serial.write("Stepping...");
          count = 0;
        }
    }
}
