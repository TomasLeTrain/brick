#include "src/subsystems/led.h"

LED leds;

TaskHandle_t motor_task, general_task;

void motor_update(void* param);
void general_update(void* param);

void setup() {
  Serial.begin();

  // add tasks
  xTaskCreatePinnedToCore(
      motor_update, // the function
      "motor_update", // name the task
      10000,     // stack depth
      NULL,      // params
      1,         // priority
      &motor_task,  // task handle (if we want to interact with the task)
      0          // core affinity - runs in core 0
  );
  xTaskCreatePinnedToCore(
      general_update, // the function
      "general_update", // name the task
      10000,     // stack depth
      NULL,      // params
      1,         // priority
      &general_task,  // task handle (if we want to interact with the task)
      1          // core affinity - runs in core 1
  );
}

// TODO: could probably just run code in loop and it would be fine?
void loop() {
  //https://forum.arduino.cc/t/esp32-using-2nd-core/931913/8
  // is bad to use loop directly??
  vTaskDelete(nullptr);
}

void motor_update(void* param){
  while(true){
   Serial.printf("motor update %d\n", micros());
   delay(1);
  }
  // digitalWrite(ledPin, HIGH);
  // delay(1000);
  // digitalWrite(ledPin, LOW);
  // delay(1000);
}

void general_update(void* param){
  int thing = 0;
  while(true){
   Serial.printf("general update %d\n", micros());

  // leds.leds_on_gradient(thing);
  leds.leds_on_rgb(0,0,15);
  thing+=5;

   delay(100);
  }
}
