/** 
 * @file pendulum-controller.ino
 * @brief Inverted pendulum kit controller
 * @author Shawn Hymel
 * @date 2023-08-05
 * 
 * @details
 * Use JSON strings to control the stepper motor and read from the encoder
 * on the STMicroelectronics inverted pendulum kit (STEVAL-EDUKIT01). Used
 * for designing controllers and reinforcement learning AI agents in Python
 * (or other high-level languages).
 *
 * @copyright
 * Zero-Clause BSD
 * 
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
 * FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "RotaryEncoder.h"
#include "L6474.h"
#include "control-comms.hpp"

/******************************************************************************
 * Constants and globals
 */

// Pin definitions
const int LED_PIN = LED_BUILTIN;
const int ENC_A_PIN = D4;     // Green wire
const int ENC_B_PIN = D5;     // White wire
const int STP_FLAG_IRQ_PIN = D2;
const int STP_STBY_RST_PIN = D8;
const int STP_DIR_PIN = D7;
const int STP_PWM_PIN = D9;
const int8_t STP_SPI_CS_PIN = D10;
const int8_t STP_SPI_MOSI_PIN = D11;
const int8_t STP_SPI_MISO_PIN = D12;
const int8_t STP_SPI_SCK_PIN = D13;

// Communication constants
static constexpr size_t NUM_ACTIONS = 1;
static constexpr size_t NUM_OBS = 2;
static const unsigned int STATUS_OK = 0;
static const unsigned int STATUS_STP_MOVING = 1;
static const unsigned int CMD_RESET = 0;
static const unsigned int CMD_MOVE_TO = 1;
static const unsigned int CMD_MOVE_BY = 2;

// Stepper and encoder constants
const int ENC_STEPS_PER_ROTATION = 1200;
const int STP_STEPS_PER_ROTATION = 400 * 8; // 400 steps, 1/8 microstep

// Stepper config
L6474_init_t stepper_config = {
  160,                              // Acceleration rate in pps^2. Range: (0..+inf)
  160,                              // Deceleration rate in pps^2. Range: (0..+inf)
  1000,                             // Maximum speed in pps. Range: (30..10000]
  800,                              // Minimum speed in pps. Range: [30..10000)
  250,                              // Torque regulation current in mA. Range: 31.25mA to 4000mA
  L6474_OCD_TH_750mA,               // Overcurrent threshold (OCD_TH register)
  L6474_CONFIG_OC_SD_ENABLE,        // Overcurrent shutwdown (OC_SD field of CONFIG register)
  L6474_CONFIG_EN_TQREG_TVAL_USED,  // Torque regulation method (EN_TQREG field of CONFIG register)
  L6474_STEP_SEL_1_8,               // Step selection (STEP_SEL field of STEP_MODE register)
  L6474_SYNC_SEL_1_2,               // Sync selection (SYNC_SEL field of STEP_MODE register)
  L6474_FAST_STEP_12us,             // Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us
  L6474_TOFF_FAST_8us,              // Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us
  3,                                // Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us
  21,                               // Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us
  L6474_CONFIG_TOFF_044us,          // Target Switching Period (field TOFF of CONFIG register)
  L6474_CONFIG_SR_320V_us,          // Slew rate (POW_SR field of CONFIG register)
  L6474_CONFIG_INT_16MHZ,           // Clock setting (OSC_CLK_SEL field of CONFIG register)
  L6474_ALARM_EN_OVERCURRENT |
  L6474_ALARM_EN_THERMAL_SHUTDOWN |
  L6474_ALARM_EN_THERMAL_WARNING |
  L6474_ALARM_EN_UNDERVOLTAGE |
  L6474_ALARM_EN_SW_TURN_ON |
  L6474_ALARM_EN_WRONG_NPERF_CMD    // Alarm (ALARM_EN register)
};

// Globals
RotaryEncoder *encoder = nullptr;
volatile int led_state = 0;
SPIClass dev_spi(STP_SPI_MOSI_PIN, STP_SPI_MISO_PIN, STP_SPI_SCK_PIN);
L6474 *stepper;
ControlComms ctrl;

/******************************************************************************
 * Interrupt service routines (ISRs)
 */

// Stepper interrupt service routine (timer)
void stepperISR(void) {

  // Set ISR flag in stepper controller
  stepper->isr_flag = TRUE;

  // Read the status register
  unsigned int status = stepper->get_status();

  // If NOTPERF_CMD flag is set, the SPI command cannot be performed
  if ((status & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
    Serial.println("    WARNING: FLAG interrupt triggered. Non-performable " \
                    "command detected when updating L6474's registers while " \
                    "not in HiZ state.");
  }

  // Reset ISR flag in stepper controller
  stepper->isr_flag = FALSE;
}

// Encoder interrupt service routine (pin change): check state
void encoderISR() {
  encoder->tick();
}

/******************************************************************************
 * Functions
 */

// Get the angle of the encoder in degrees (0 is starting position)
float get_encoder_angle() {
  
  int pos, dir;
  float deg = 0.0;

  // Get position and direction
  pos = encoder->getPosition();
  dir = (int)encoder->getDirection();

  // Convert to degrees
  pos = pos % ENC_STEPS_PER_ROTATION;
  pos = pos >= 0 ? pos : pos + ENC_STEPS_PER_ROTATION;
  deg = (float)pos * (360.0 / ENC_STEPS_PER_ROTATION);

  return deg;
}

// Get the position of the stepper motor (in degrees)
float get_stepper_angle() {
  
  int pos;
  float deg = 0.0;
  
  // Get stepper position (in number of steps)
  pos = stepper->get_position();

  // Convert to degrees
  pos = pos % STP_STEPS_PER_ROTATION;
  pos = pos >= 0 ? pos : pos + STP_STEPS_PER_ROTATION;
  deg = (float)pos * (360.0 / STP_STEPS_PER_ROTATION);

  return deg;
}

// Tell the stepper motor to move to a particular angle in degrees
void move_stepper_to(float deg) {

  int steps = (int)(deg * STP_STEPS_PER_ROTATION / 360.0);

  // Tell stepper motor to move
  stepper->go_to(steps);
}

// Tell the stepper motor to move by a particular angle in degrees
void move_stepper_by(float deg) {

  StepperMotor::direction_t stp_dir = StepperMotor::FWD;
  int steps = (int)(deg * STP_STEPS_PER_ROTATION / 360.0);

  // Use direction and absolute step counts
  if (steps < 0) {
    steps = -1 * steps;
    stp_dir = StepperMotor::BWD;
  }

  // Tell stepper motor to move
  stepper->move(stp_dir, steps);
}

/******************************************************************************
 * Main
 */

void setup() {

  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  // Initialize our communication interface
  Serial.begin(115200);
  ctrl.init(Serial, ControlComms::DEBUG_NONE);

  // Configure encoder
  encoder = new RotaryEncoder(
    ENC_A_PIN, 
    ENC_B_PIN, 
    RotaryEncoder::LatchMode::TWO03
  );

  // Configure encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

  // Initialize stepper motor control
  stepper = new L6474(
    STP_FLAG_IRQ_PIN,
    STP_STBY_RST_PIN,
    STP_DIR_PIN,
    STP_PWM_PIN,
    STP_SPI_CS_PIN,
    &dev_spi
  );
  if (stepper->init(&stepper_config) != COMPONENT_OK) {
    Serial.println("ERROR: Could not initialize stepper driver");
    while(1);
  }

  // Attach and enable stepper motor interrupt handlers
  stepper->attach_flag_irq(&stepperISR);
  stepper->enable_flag_irq();

  // Set current position as home
  stepper->set_home();
}

void loop() {

  int command;
  float action[NUM_ACTIONS];
  int status;
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;

  // Receive
  rx_code = ctrl.receive_action<NUM_ACTIONS>(&command, action);
  if (rx_code == ControlComms::OK) {

    // Move the stepper as requested
    switch (command) {
      case CMD_RESET:
        move_stepper_to(0.0);
        break;
      case CMD_MOVE_TO:
        move_stepper_to(action[0]);
        break;
      case CMD_MOVE_BY:
        move_stepper_by(action[0]);
        break;
      default:
        break;
    }

    // Read encoder and stepper angles (in degrees)
    observation[0] = get_encoder_angle();
    observation[1] = get_stepper_angle();

    // Determine motor status
    if (stepper->get_device_state() != INACTIVE) {
      status = STATUS_STP_MOVING;
    } else {
      status = STATUS_OK;
    }

    // Send back observation
    ctrl.send_observation(status, millis(), false, observation, NUM_OBS);
  
  // Handle receiver error (ignore "RX_EMPTY" case)
  } else if (rx_code == ControlComms::ERROR) {
    Serial.println("Error receiving actions");
  }
}
