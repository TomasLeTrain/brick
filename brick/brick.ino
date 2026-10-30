#include "robot.h"

Robot robot;

TaskHandle_t motor_task, general_task;
void motor_update(void* param);
void general_update(void* param);

void setup() {
  Serial.begin();

  robot.begin();

  // now do everything else
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

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

  robot.arming_sequence();
}

// TODO: could probably just run code in loop and it would be fine?
void loop() {
  //https://forum.arduino.cc/t/esp32-using-2nd-core/931913/8
  // is bad to use loop directly??
  vTaskDelete(nullptr);
}


void motor_update(void* param){
  while(true){
    robot.motor_update();
    vTaskDelay(1); 
  }
}

void general_update(void* param){
  int thing = 0;
  while(true){
    robot.general_update();

  vTaskDelay(10);

  }
}
