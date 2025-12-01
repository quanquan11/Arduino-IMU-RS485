#define NODE_ID 0
#define FILTER_ALPHA 0.98
#define TRANSMIT_RATE 10

// Test mode: Set to true to continuously send data to Serial without waiting for RS485 poll
// Set to false for normal RS485 operation
#define TEST_MODE false

// Arduino Pro Micro compatible pins
#define DE_PIN  15   // Dummy pin (NOT connected to RS485 module - only if module has auto-direction)
#define RX_PIN  1   // SoftwareSerial RX (connect to RS485 module RO pin)
#define TX_PIN  0   // SoftwareSerial TX (connect to RS485 module DI pin)