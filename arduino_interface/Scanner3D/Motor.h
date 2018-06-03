#include <Arduino.h>
#include <stdint.h>

#ifndef Motor_H
#define Motor_H

/****** PINS Motor *****/
#define analog_obp A5
#define PIN_OBP 4
#define PIN_FAULT 5
#define PIN_ENABLE 6
#define PIN_M0 7
#define PIN_M1 8
#define PIN_M2 9
#define PIN_RESET 10
#define PIN_SLEEP 11
#define PIN_STEP 12
#define PIN_DIR 13

class Motor {
  private:
    const int opb_threshold = 24;
    int opb_value;
    bool at_home_position;
  public:
    Motor();
    void init();
    void Step();
    void Home();
};

#endif
