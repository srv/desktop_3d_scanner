
#include "Motor.h"

/*************** Inicializaciones **************/

Motor::Motor(){

//Configuramos todos los pines del driver poder actuar sobre ellos
	//Configurar la resolución de giro
  pinMode(PIN_M0, OUTPUT);
	pinMode(PIN_M1, OUTPUT);
	pinMode(PIN_M2, OUTPUT);
	//Pin de fallo
	pinMode(PIN_FAULT, INPUT);
	//Habilitazión del driver
	pinMode(PIN_ENABLE, OUTPUT);
	//Reset del driver
	pinMode(PIN_RESET, OUTPUT); 
	//Configuración en modo SLEEP
	pinMode(PIN_SLEEP, OUTPUT);
	//Pin para realizar un paso
	pinMode(PIN_STEP, OUTPUT);
	//Dirección de giro: horario/antihorario
	pinMode(PIN_DIR, OUTPUT);


}

void Motor::init(){
	//Se configura a una resolucion de 1/32 step
    digitalWrite(PIN_M0, HIGH);
    digitalWrite(PIN_M1, HIGH);
    digitalWrite(PIN_M2, HIGH);
}

/*************** Control del motor **************/

void Motor::Step(){
	digitalWrite(PIN_STEP, HIGH);  
    delayMicroseconds(2);       
    digitalWrite(PIN_STEP, LOW);
}

/***************Home method********************/
void Motor::Home(){
    int i;
    opb_value = analogRead(analog_obp);
    while(opb_value > opb_threshold && i < 40) {
        Step();
        opb_value=analogRead(analog_obp);
        delay(5);
        if(opb_value > opb_threshold) {
            i++;
        }
    }
    while(opb_value < opb_threshold) {
        Step();
        opb_value=analogRead(analog_obp);
        delay(2);
    }
}
