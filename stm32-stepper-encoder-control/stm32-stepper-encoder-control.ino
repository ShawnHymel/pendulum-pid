/**
 * STM32 Stepper Motor and Encoder Control
 *
 * Run this on an STM32 Nucleo board to control a stepper motor and read from
 * a quadrature rotary encoder. Commands are given over serial.
 * 
 * Stepper motor driver: X-NUCLEO-IHM01A1
 * Quadrature encoder: LPD3806-600BM-G5-24C
 *
 * Author: Shawn Hymel
 * Date: July 11, 2023
 * License: 0BSD (https://opensource.org/license/0bsd/)
 */

#include "RotaryEncoder.h"
#include "L6474.h"

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

// Constants
const int ENC_STEPS_PER_ROTATION = 1200;
const int STP_STEPS_PER_ROTATION = 400 * 8; // 400 steps, 1/8 microstep

// Message constants
const uint8_t TX_LEN = 6;
const uint8_t TX_SOF = 0xA5;
const uint8_t TX_EOF = 0x0A;
const uint8_t TX_CMD_ACK = 0x01;
const uint8_t TX_CMD_NAK = 0x02;
const uint8_t TX_CMD_GET_ENC = 0x10;
const uint8_t TX_CMD_GET_STP = 0x20;
const uint8_t TX_CMD_SET_STP_HOME = 0x21;
const uint8_t TX_CMD_MOV_STP_HOME = 0x22;
const uint8_t TX_CMD_MOV_STP_BY = 0x23;
const uint8_t TX_CMD_MOV_STP_TO = 0x24;

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
// Dir: >= 1 is forward, everything else is backward
void move_stepper(int dir, float deg) {

  StepperMotor::direction_t stp_dir = StepperMotor::FWD;
  int steps = (int)(deg * STP_STEPS_PER_ROTATION / 360.0);

  // Assign the direction
  if (dir <= 0) {
    stp_dir = StepperMotor::BWD;
  }

  // Tell stepper motor to move
  stepper->move(stp_dir, steps);
}

// Calculate the checksum of a message
uint8_t calculate_checksum(uint8_t msg[], unsigned int len) {

  uint8_t checksum = 0;

  for (unsigned int i = 0; i < len; i++) {
    checksum ^= msg[i];
  }

  return checksum;
}

/******************************************************************************
 * Main
 */

void setup() {

  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  // Pour some serial
  Serial.begin(115200);

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

  float enc_deg;
  float stp_deg;
  static int stp_dir = 0;
  String str;
  int len = 0;
  unsigned int byte_counter = 0;
  char msg[10];

  // See if there's a message for us
  if (Serial.available() > 0) {

    // Read entire message
    while (Serial.available() > 0) {
      // TODO: RX not dependent on EOF (as any byte could be '\n')
      // Include a timeout
      // Check for correct SOF
      // Only need to read 6 bytes (including SOF and EOF)
    }

    str = Serial.readStringUntil(EOF);
    
    // Convert String to character array
    len = str.length();
    str.toCharArray(msg, len);

    // Parse the message
    if (msg[0] == TX_SOF) {
      for (int i = 1; i < len; i++) {
        Serial.print(msg[i]);
      }
      Serial.println();
    }
  }

  // In case you use an RTOS, let other things run
  yield();

  // // Move the stepper
  // stp_dir = !stp_dir;
  // move_stepper(stp_dir, 90.0);

  // // Read encoder and stepper angles (in degrees)
  // enc_deg = get_encoder_angle();
  // stp_deg = get_stepper_angle();

  // // Print all the things
  // Serial.print("Encoder (deg): ");
  // Serial.print(enc_deg, 2);
  // Serial.print(" | Stepper (deg): ");
  // Serial.println(stp_deg, 2);

  // // TODO: non-blocking wait
  // stepper->wait_while_active();
}
