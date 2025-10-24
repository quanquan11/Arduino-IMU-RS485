#define NODE_ID 0
#define FILTER_ALPHA 0.98
#define TRANSMIT_RATE 10

// Arduino Pro Micro compatible pins
#define DE_PIN  15   // Dummy pin (NOT connected to RS485 module - only if module has auto-direction)
#define RX_PIN  1   // SoftwareSerial RX (connect to RS485 module RO pin)
#define TX_PIN  0   // SoftwareSerial TX (connect to RS485 module DI pin)