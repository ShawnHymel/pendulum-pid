// TODO: 
// * Use ArduinoJSON to parse received messages
//   - Make doc <size> dynamic (or set on instantiation)
// * Create send_observation() function
// * Write matching functions in Python
// * Tie to stepper and encoder controls
// * ???
// * Profit

#include "control-comms.h"

static constexpr unsigned int NUM_ACTIONS = 5;

ControlComms ctrl;

void setup() {

  // Initialize our communication interface
  Serial.begin(115200);
  ctrl.init(Serial, 115200, ControlComms::DEBUG_ERROR);
}

void loop() {

  float actions[NUM_ACTIONS];
  ControlComms::StatusCode rx_code;

  // Receive
  rx_code = ctrl.receive_actions<NUM_ACTIONS>(actions);
  if (rx_code != ControlComms::OK) {
    Serial.println("Error receiving");
  }

  // In case you use an RTOS, let other things run
  delay(10);
  // yield();

}
