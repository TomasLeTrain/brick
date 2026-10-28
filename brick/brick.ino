// #include "src/subsystems/led.h"
#include "src/lib/AlfredoDShot.h"

AlfredoDShot esc;

// LED leds;

TaskHandle_t motor_task, general_task;

void motor_update(void* param);
void general_update(void* param);

bool arming = true;
float throttle = 0.1;
int command = 0;

void setup() {
  // AlfredoDShot::releaseBootloader(4);   // must be first, see Troubleshooting
  esc.begin(GPIO_NUM_4, DSHOT600, /*bidirectional=*/false, /*motorPoles=*/14);

  Serial.begin();

  // wait for armed
  // while(!esc.isArmed()){
  //   delay(10);
  // }


  // send non-zero for a lil bit
  // int curr = millis();
  // while(millis() - curr < 1000){
  //   esc.send(50);
  //   delay(10);
  // }

  // // then send zero again
  // curr = millis();
  // while(millis() - curr < 1000){
  //   esc.send(0);
  //   delay(10);
  // }

  // now do everything else

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // delay(2000);
//  wait for esc
//   while(!esc.isArmed()){
//     Serial.println("waiting");
//     delay(10);
// }
  
  // delay even more
  // delay(100);
  // int count = 0;
  // while(count < 3000){
  //   esc.send(0);
  //   delay(1);
  //   count++;
  // }


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
    vTaskDelay(1); 
  }
}

void general_update(void* param){
  int thing = 0;
  while(true){
  //  Serial.printf("general update %d\n", micros());

  // leds.leds_on_gradient(thing);
  // leds.leds_on_rgb(0,0,15);
  // thing+=5;

  //  delay(100);

  vTaskDelay(10);

  }
}
