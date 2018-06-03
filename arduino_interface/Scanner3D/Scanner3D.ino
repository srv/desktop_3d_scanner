/**************************************************************
    Programa para controlar el Motor PaP del Scanner 3D
    Autor: Marta Pons Nieto
**************************************************************/
#include <stdio.h>
#include "Motor.h"


/******* VARIABLES CONECTADAS A MATLAB *******/
volatile uint8_t NumSteps;
bool PerformStep;      //Recibe la orden de realizar un paso
bool StepDone = false; //Indica si ya se ha realizado el paso ordenado

Motor Motor;

const int ledpin1 = 12;
const int ledpin2 = 11;
const int ledpin3 = 10;
const int ledpin4 = 9;
const int ledpin5 = 8;
const int ledpin6 = 7;
const int ledpin7 = 6;
const int ledpin8 = 5;
uint8_t instruction;

void setup() {
  Motor.init();
  Serial.begin(9600);
  pinMode( ledpin1, OUTPUT);
  pinMode( ledpin2, OUTPUT);
  /*pinMode( ledpin3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);*/
}

void loop()
{
  if (Serial.available() > 0)
  {
    instruction = Serial.read();

    if  (instruction == 1)             // MATLAB wants to perform a step
    {
      //Motor.Step();
      digitalWrite(ledpin1, LOW);
      delay(500);
      Serial.println(instruction + 2); //StepDone
    } else {
      digitalWrite(ledpin1, HIGH);
    }
    if  (instruction == 2)             // MATLAB wants Homming
    {
      digitalWrite(ledpin2, LOW);
      delay(500);
      Serial.println(instruction + 2); //Motor at home position
    } else {
      digitalWrite(ledpin2, HIGH);
    }
    delay(500);
  }
}
