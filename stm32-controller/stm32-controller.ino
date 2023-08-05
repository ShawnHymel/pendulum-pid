// TODO: 
// * Write matching functions in Python
// * Tie to stepper and encoder controls
// * ???
// * Profit

#include "control-comms.hpp"

static constexpr size_t NUM_ACTIONS = 2;
static constexpr size_t NUM_OBS = 3;

ControlComms ctrl;

void setup() {

  // Initialize our communication interface
  Serial.begin(115200);
  ctrl.init(Serial);
}

void loop() {

  float action[NUM_ACTIONS];
  float observation[NUM_OBS] = {3, 4, 5};
  ControlComms::StatusCode rx_code;

  // Receive
  rx_code = ctrl.receive_action<NUM_ACTIONS>(action);
  if (rx_code == ControlComms::OK) {

    // Send back observation
    ctrl.send_observation(0, millis(), false, observation, NUM_OBS);
  
  // Handle receiver error (ignore "RX_EMPTY" case)
  } else if (rx_code == ControlComms::ERROR) {
    Serial.println("Error receiving actions");
  }

  // In case you use an RTOS, let other things run
  // delay(10);
  // yield();

}
