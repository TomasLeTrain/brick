#include "src/subsystems/led.h"
#include "src/lib/AlfredoDShot.h"
#include "robot.h"

AlfredoDShot esc;

Robot robot;
LED leds;

TaskHandle_t motor_task, general_task;
void motor_update(void* param);
void general_update(void* param);

bool arming = true;
float throttle = 0.1;
int command = 0;

void setup() {
  esc.begin(GPIO_NUM_4, DSHOT600, false, 14);

  Serial.begin();

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

  // after tasks have happened, do arming sequence

  // delay(1300);
  command = 0;
  delay(3400);
  // command = 80;
  // delay(4000);
  // command = 0;
  // delay(300);
  arming = false;
}

// TODO: could probably just run code in loop and it would be fine?
void loop() {
  //https://forum.arduino.cc/t/esp32-using-2nd-core/931913/8
  // is bad to use loop directly??
  vTaskDelete(nullptr);
}


void motor_update(void* param){
  while(true){
    if(arming){
      esc.send(command);
    }else{
      esc.sendThrottle(throttle);
    }

    if(esc.isArmed()){
      digitalWrite(LED_BUILTIN, LOW);
    }
    robot.motor_update();
    vTaskDelay(1); 
  }
}

void general_update(void* param){
  int thing = 0;
  while(true){
    robot.general_update();
  //  Serial.printf("general update %d\n", micros());

  // leds.leds_on_gradient(thing);
  leds.leds_on_rgb(0,0,15);
  // thing+=5;

  vTaskDelay(10);

  }
}
