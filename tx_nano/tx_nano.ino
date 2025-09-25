/*
File name     = tx_nano.ino
Author        = Leo Cabezas
Credits       = Credit to Kieran for implementing getHammingParityBits().
Project       = Project 1 (IR data transmission system)
Course        = EECS 541
Description   = TX code for the Arduino Nano.
*/

// ============================= IMPORTED LIBRARIES (DO NOT MODIFY)  =========================
#include <stdint.h> // For int8_t, uint8_t, uint16_t, and uint32_t.
#include <string.h> // For memcpy() and size_t.
#include <math.h>   // For ceil().

// ============================== TX PARAMETERS (MODIFY AS NEEDED) ===========================
// --------------- Pertaining to message to be transmitted ---------------
const size_t MSG_LEN = 11;  // 11; 21; 27
char message_str[MSG_LEN] = "hello_world";  // hello_world; embedded_systems_rock; c_is_the_best_language_ever
// --------------- Pertaining to data transmission ---------------
const uint32_t pulse_width_us = 500;  // Must be below 1e6 / f_carrier_Hz. 
const uint32_t f_carrier_Hz = 1200;   // 1200; 600; 2400
const uint32_t f_deviation_Hz = 50;   // 50; 10; 300
const uint16_t start_sync_frame = 0b1111111111111111;
const uint16_t end_sync_frame = 0b0011001100110011;
// --------------- Pertaining to error injection ---------------
const bool enable_error_injection = false;
const size_t TARGET_COUNT = 4;
const size_t target_frame_idx1[TARGET_COUNT]= {0,1,2,3};
const size_t target_bit_idx1[TARGET_COUNT]= {3,3,3,3};
const size_t target_frame_idx2[TARGET_COUNT]= {0,1,2,3};
const size_t target_bit_idx2[TARGET_COUNT]= {8,8,8,8};
// --------------- Pertaining to Arduino Nano setup ---------------
const uint8_t TXpin = 7;

// ============================== OTHER VARIABLES (DO NOT MODIFY) ============================
// --------------- Useful constants ---------------
const size_t FRAME_LEN = 8 * sizeof(uint16_t);
// --------------- Pertaining to Arduino control loop ---------------
uint32_t message_tx_us[MSG_LEN + 2][FRAME_LEN];

// ================================= TX FUNCTION DEFINITIONS =================================
uint32_t instantPeriod_us(
  uint32_t f_carrier_Hz, 
  uint32_t f_deviation_Hz, 
  uint8_t bit
){
  int8_t signed_bit = (bit == 0 ? -1 : 1);
  return (uint32_t)(ceil(1000000.0 / (f_carrier_Hz + f_deviation_Hz * signed_bit)));
}

uint8_t getHammingParityBits(
  uint8_t nibble
){
  // Variant of Hamming(7, 4).
  uint8_t d1 = (nibble >> 0) & 1;
  uint8_t d2 = (nibble >> 1) & 1;
  uint8_t d3 = (nibble >> 2) & 1;
  uint8_t d4 = (nibble >> 3) & 1;

  uint8_t p1 = d1 ^ d2 ^ d4;  // First parity bit
  uint8_t p2 = d1 ^ d3 ^ d4;  // Second parity bit
  uint8_t p3 = d2 ^ d3 ^ d4;  // Third parity bit

  return (p1 << 2) | (p2 << 1) | p3;
}

uint16_t injectError(
  size_t frame_idx,
  size_t target_frame_idx1[TARGET_COUNT],
  size_t target_bit_idx1[TARGET_COUNT],
  size_t target_frame_idx2[TARGET_COUNT],
  size_t target_bit_idx2[TARGET_COUNT]
){
  uint16_t mask = 0;
  for (size_t i = 0; i < TARGET_COUNT; i++) {
    if (frame_idx == target_frame_idx1[i]) {
      mask |= ((uint16_t)(1 << FRAME_LEN - 1) >> target_bit_idx1[i]);
    }
  }
  for (size_t i = 0; i < TARGET_COUNT; i++) {
    if (frame_idx == target_frame_idx2[i]) {
      mask |= ((uint16_t)(1 << FRAME_LEN - 1) >> target_bit_idx2[i]);
    }
  }
  return mask;
}

uint16_t packetToFrame(
  char packet
) {
  uint16_t start_bit = 1; // Must be 0 or 1.
  uint16_t end_bit = 0;   // Must be 0 or 1.

  uint8_t nibble1 = ((packet & 0b11110000) >> 4);
  uint8_t nibble2 = (packet & 0b00001111);

  uint16_t nibble1_hamming = (uint16_t)getHammingParityBits(nibble1);
  uint16_t nibble2_hamming = (uint16_t)getHammingParityBits(nibble2);

  uint16_t frame = 0;
  frame |= (start_bit << 15);         // Set start bit.
  frame |= (end_bit << 0);            // Set end bit.
  frame |= ((uint16_t)nibble1 << 11); // Set first nibble.
  frame |= (nibble1_hamming << 8);    // Set first nibble's parity bits.
  frame |= ((uint16_t)nibble2 << 4);  // Set second nibble.
  frame |= (nibble2_hamming << 1);    // Set second nibble's parity bits.
  return frame;
}

void encodeFrame_us(
  uint16_t frame, 
  uint32_t frame_tx_us[FRAME_LEN], 
  uint32_t f_carrier_Hz, 
  uint32_t f_deviation_Hz
){
  uint32_t instant_period_us = 0;
  for (size_t i=0; i < FRAME_LEN; i++){
    instant_period_us = instantPeriod_us(f_carrier_Hz, f_deviation_Hz, (frame >> FRAME_LEN - 1 - i) & 1);
    frame_tx_us[i] = instant_period_us;
  }
}

void encodeMessage_us(
  char message_str[MSG_LEN],
  uint32_t message_tx_us[MSG_LEN + 1][FRAME_LEN],
  uint32_t f_carrier_Hz,
  uint32_t f_deviation_Hz,
  uint16_t start_sync_frame,
  uint16_t end_sync_frame,
  size_t target_frame_idx1[TARGET_COUNT],
  size_t target_bit_idx1[TARGET_COUNT],
  size_t target_frame_idx2[TARGET_COUNT],
  size_t target_bit_idx2[TARGET_COUNT]
){
  uint16_t frame = 0;
  uint32_t frame_tx_us[FRAME_LEN];

  encodeFrame_us(start_sync_frame, frame_tx_us, f_carrier_Hz, f_deviation_Hz);
  memcpy(message_tx_us[0], frame_tx_us, sizeof(uint32_t) * FRAME_LEN);
  
  for (size_t i=0; i < MSG_LEN; i++){
    frame = packetToFrame(message_str[i]);
    if (enable_error_injection){
      frame ^= injectError(i, target_frame_idx1, target_bit_idx1, 
                          target_frame_idx2, target_bit_idx2);
    }
    encodeFrame_us(frame, frame_tx_us, f_carrier_Hz, f_deviation_Hz);
    memcpy(message_tx_us[i + 1], frame_tx_us, sizeof(uint32_t) * FRAME_LEN);
  }

  encodeFrame_us(end_sync_frame, frame_tx_us, f_carrier_Hz, f_deviation_Hz);
  memcpy(message_tx_us[MSG_LEN + 1], frame_tx_us, sizeof(uint32_t) * FRAME_LEN);
}

void sendMessage(
  uint32_t message_tx_us[MSG_LEN + 2][FRAME_LEN],
  uint32_t pulse_width_us,
  uint8_t TXpin
){
  for (size_t i=0; i < MSG_LEN + 2; i++){
    for (size_t j=0; j < FRAME_LEN; j++){
      digitalWrite(TXpin, HIGH);
      delayMicroseconds(pulse_width_us);
      digitalWrite(TXpin, LOW);
      delayMicroseconds(message_tx_us[i][j] - pulse_width_us);
    }
    // if (i != MSG_LEN + 1){delayMicroseconds(2000);}  // Uncomment to clearly visualize frames in the oscilloscope
  }
  digitalWrite(TXpin, HIGH);
  delayMicroseconds(pulse_width_us);
  digitalWrite(TXpin, LOW);
}

// ================================= ARDUINO TX CONTROL LOOP =================================
void setup(){
  encodeMessage_us(message_str, message_tx_us, f_carrier_Hz, f_deviation_Hz, 
                   start_sync_frame, end_sync_frame, target_frame_idx1, target_bit_idx1, 
                   target_frame_idx2, target_bit_idx2);
  pinMode(TXpin, OUTPUT);
  delay(500);
}

void loop(){
  sendMessage(message_tx_us, pulse_width_us, TXpin);  // To send only once, move to setup();
  delayMicroseconds(2000);  // 600 min., 1000 works, 2000 most stable, 8000 for waveform
}
