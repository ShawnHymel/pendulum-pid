// TODO: 
// * Use ArduinoJSON to parse received messages
// * Create send_observation() function
// * Write matching functions in Python
// * Tie to stepper and encoder controls
// * ???
// * Profit

#include "control-comms.h"

#define NUM_ACTIONS 5

Control ctrl;

void setup() {

  // Initialize our communication interface
  Serial.begin(115200);
  ctrl.init(Serial);
}

void loop() {

  float actions[NUM_ACTIONS];
  int num_actions_received = 0;

  // Receive
  num_actions_received = ctrl.receive_actions(actions, NUM_ACTIONS);
  if (num_actions_received < 0) {
    Serial.println("Error receiving");
  }

  // In case you use an RTOS, let other things run
  yield();

}
