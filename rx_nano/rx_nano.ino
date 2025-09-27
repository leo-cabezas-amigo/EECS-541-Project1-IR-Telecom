/*
File name     = rx_nano.ino
Author        = Leo Cabezas
Project       = Project 1 (IR data transmission system)
Course        = EECS 541
Description   = RX code for the Arduino Nano.
*/

// ============================= IMPORTED LIBRARIES (DO NOT MODIFY)  =========================
#include <stdint.h> // For uint32_t.
#include <math.h>   // For ceil().

// ============================== RX PARAMETERS (MODIFY AS NEEDED) ===========================
// --------------- Pertaining to message to be received ---------------
const size_t MAX_MSG_LEN =  133;      // Maximum permitted message length (in char).
// --------------- Pertaining to data reception ---------------
const uint32_t f_carrier_Hz = 1200;   // Carrier frequency.
const uint32_t f_deviation_Hz = 50;  // Deviation frequency (up/down) from f_carrier_Hz.
const uint16_t start_sync_frame = 0b1111111111111111;
const uint16_t end_sync_frame = 0b0011001100110011;
// --------------- Pertaining to error correction ---------------
const bool enable_error_correction = false;
// --------------- Pertaining to Arduino Nano setup ---------------
const uint8_t RXpin = 2;

// ============================== OTHER VARIABLES (DO NOT MODIFY) ============================
// --------------- Useful constants ---------------
const size_t FRAME_LEN = 8 * sizeof(uint16_t);
const uint8_t BITS_PER_FRAME = 16;
const uint8_t DATA_BITS_PER_FRAME = 8;

// ============================== GLOBAL VARIABLES (DO NOT MODIFY) ===========================
// --------------- Pertaining to risingEdgeHandler() ---------------
volatile bool pulse_detected = false; // Flag to indicate a rising edge has been detected
volatile uint32_t last_rising_edge_us = 0;  // Store the time of the last rising edge
volatile uint32_t curr_rising_edge_us = 0;  // Store the time of the current rising edge
volatile uint32_t instant_period_us = 0; // Time elapsed (in us) between consecutive rising edges
// --------------- Pertaining to Arduino Nano setup and loop ---------------
char* message_buffer;
uint16_t frame_buffer = 0;
uint32_t frame_count = 0;
uint8_t frame_bit_count = 0;
bool start_sync_frame_detected = false;
bool end_sync_frame_detected = false;
// --------------- Pertaining to error correction statistics ---------------
uint32_t msg_data_bit_count = 0;
uint32_t msg_error_count = 0;
double msg_error_rate = 0.0;
// --------------- Pertaining to frequency statistics ---------------
uint32_t msg_bit_count = 0;
uint32_t msg_cumulative_f_Hz = 0;
uint32_t msg_average_f_Hz = 0;
uint32_t msg_max_f_Hz = 0;
uint32_t msg_min_f_Hz = UINT32_MAX;
// --------------- Pertaining to debug information ---------------
const size_t DEBUG_BUF_LEN = 128;
char debug_buffer[DEBUG_BUF_LEN];
size_t debug_buf_idx = 0;

// ================================= RX FUNCTION DEFINITIONS =================================
// --------------- Bit detection functions ---------------
void risingEdgeHandler(){
  curr_rising_edge_us = micros();
  if (last_rising_edge_us > 0){
    instant_period_us = curr_rising_edge_us - last_rising_edge_us;
    pulse_detected = true;
  }
  last_rising_edge_us = curr_rising_edge_us;
}

uint16_t getBitFromPeriod_us(
  uint32_t instant_period_us, 
  uint32_t f_carrier_Hz, 
  uint32_t f_deviation_Hz
){
  // From TX formula for instantPeriod_us.
  double bit_calc = ((1000000.0 / instant_period_us) - f_carrier_Hz) / f_deviation_Hz; 
  return (uint16_t)(bit_calc > 0 ? 1 : 0);
}

bool frameDetect(
  uint16_t buffer
){
  bool start_bit_detected = (((buffer >> FRAME_LEN - 1) & 1) == 1);
  bool end_bit_detected = ((buffer & 1) == 0);
  if (start_bit_detected && end_bit_detected){
    return true;
  } else {
    return false;
  }
}

// --------------- Error detection and correction functions ---------------
bool getHammingCorrectedNibble(
  uint8_t nibble, 
  uint8_t parity_bits, 
  uint8_t& nibble_corrected
){
  uint8_t d1 = (nibble >> 0) & 1;
  uint8_t d2 = (nibble >> 1) & 1;
  uint8_t d3 = (nibble >> 2) & 1;
  uint8_t d4 = (nibble >> 3) & 1;

  uint8_t p1 = (parity_bits >> 2) & 1;
  uint8_t p2 = (parity_bits >> 1) & 1;
  uint8_t p3 = (parity_bits >> 0) & 1;

  uint8_t s1 = p1 ^ d1 ^ d2 ^ d4;
  uint8_t s2 = p2 ^ d1 ^ d3 ^ d4;
  uint8_t s3 = p3 ^ d2 ^ d3 ^ d4;
  uint8_t syndrome = (s3 << 2) | (s2 << 1) | s1;

  p1 ^= (syndrome == 1);
  p2 ^= (syndrome == 2);
  d1 ^= (syndrome == 3);
  p3 ^= (syndrome == 4);
  d2 ^= (syndrome == 5);
  d3 ^= (syndrome == 6);
  d4 ^= (syndrome == 7);

  nibble_corrected = (d4 << 3) | (d3 << 2) | (d2 << 1) | d1;
  return (syndrome != 0);
}

// --------------- Message processing functions ---------------
void mallocMessageBuffer(
  char** message_buffer
){
  *message_buffer = (char*)malloc(MAX_MSG_LEN * sizeof(char));
  if (*message_buffer == NULL) {
    Serial.println("WARNING: Memory allocation for 'message_buffer' failed!");
    while (true);
  }
}

void resetMessageBuffer(
  char* message_buffer,
  uint16_t& frame_buffer,
  uint32_t& frame_count,
  uint8_t& frame_bit_count
){
  memset(message_buffer, 0, MAX_MSG_LEN);
  frame_buffer = 0;
  frame_count = 0;
  frame_bit_count = 0;
}

void updateMessageBuffer(
  uint16_t frame_buffer,
  char* message_buffer,
  size_t msg_buffer_idx,
  uint32_t& msg_error_count
){
  uint16_t nibble1_mask = 0b0111100000000000;
  uint16_t nibble2_mask = 0b0000000011110000;
  uint8_t nibble1_raw = (uint8_t)((frame_buffer & nibble1_mask) >> 11);
  uint8_t nibble2_raw = (uint8_t)((frame_buffer & nibble2_mask) >> 4);

  uint16_t parity_bits1_mask = 0b0000011100000000;
  uint16_t parity_bits2_mask = 0b0000000000001110;
  uint8_t parity_bits1 = (uint8_t)((frame_buffer & parity_bits1_mask) >> 8);
  uint8_t parity_bits2 = (uint8_t)((frame_buffer & parity_bits2_mask) >> 1);

  uint8_t nibble1_corrected = 0;
  uint8_t nibble2_corrected = 0;
  msg_error_count += getHammingCorrectedNibble(nibble1_raw, parity_bits1, nibble1_corrected);
  msg_error_count += getHammingCorrectedNibble(nibble2_raw, parity_bits2, nibble2_corrected);

  char new_char;
  if (enable_error_correction){
    new_char = (char)((nibble1_corrected << 4) | nibble2_corrected);
  } else {
    new_char = (char)((nibble1_raw << 4) | nibble2_raw);
  }
  
  if (msg_buffer_idx < MAX_MSG_LEN){
    message_buffer[msg_buffer_idx] = new_char;
  }
}

void printReceivedMessage(
  char* message_buffer,
  size_t msg_len
){
  Serial.print("[msg: \"");
  Serial.write(message_buffer, msg_len);  
  Serial.print("\"]\n");
}

// --------------- Error correction statistics functions ---------------
void resetErrorCounters(
  uint32_t& msg_error_count,
  uint32_t& msg_data_bit_count,
  double& msg_error_rate
){
  msg_data_bit_count = 0;
  msg_error_count = 0;
  msg_error_rate = 0.0;
}

void updateMsgErrorRate(
  uint32_t msg_error_count,
  uint32_t& msg_data_bit_count,
  double& msg_error_rate
){
  msg_data_bit_count += (uint32_t)DATA_BITS_PER_FRAME;
  if (msg_data_bit_count > 0) {
    msg_error_rate = ((double)msg_error_count / msg_data_bit_count) * 100.0;  // multiply by 100 for percentage
  } else {
    msg_error_rate = 0.0;
  }
}

void appendMsgErrorStats(
  char debug_buffer[DEBUG_BUF_LEN],
  size_t& debug_buf_idx,
  double msg_error_rate
){
  char msg_error_rate_str[5];
  dtostrf(msg_error_rate, 0, 1, msg_error_rate_str);
  debug_buf_idx += snprintf(debug_buffer + debug_buf_idx, DEBUG_BUF_LEN - debug_buf_idx, 
    "[err%%: %s%%] ", msg_error_rate_str);
}

// --------------- Frequency statistics functions ---------------
void resetFreqCounters(
  uint32_t& msg_bit_count,
  uint32_t& msg_cumulative_f_Hz,
  uint32_t& msg_average_f_Hz,
  uint32_t& msg_max_f_Hz,
  uint32_t& msg_min_f_Hz
){
  msg_bit_count = 0;
  msg_cumulative_f_Hz = 0;
  msg_average_f_Hz = 0;
  msg_max_f_Hz = 0;
  msg_min_f_Hz = UINT32_MAX;
}

void updateMsgAvgFreq(
  uint32_t instant_period_us,
  uint32_t& msg_bit_count,
  uint32_t& msg_cumulative_f_Hz,
  uint32_t& msg_average_f_Hz
){
  uint32_t instant_f_Hz = (uint32_t)ceil(1000000.0 / instant_period_us);
  msg_bit_count += 1;
  msg_cumulative_f_Hz += instant_f_Hz;
  msg_average_f_Hz = (uint32_t)ceil((double)msg_cumulative_f_Hz / msg_bit_count);
}

void updateMsgMinMaxFreqs(
  uint32_t instant_period_us,
  uint32_t& msg_max_f_Hz,
  uint32_t& msg_min_f_Hz
){
  uint32_t instant_f_Hz = (uint32_t)ceil(1000000.0 / instant_period_us);
  if (instant_f_Hz > msg_max_f_Hz){
    msg_max_f_Hz = instant_f_Hz;
  }
  if (instant_f_Hz < msg_min_f_Hz){
    msg_min_f_Hz = instant_f_Hz;
  }
}

void appendMsgFreqStats(
  char debug_buffer[DEBUG_BUF_LEN],
  size_t& debug_buf_idx,
  uint32_t msg_average_f_Hz,
  uint32_t msg_min_f_Hz,
  uint32_t msg_max_f_Hz
){
  debug_buf_idx += snprintf(debug_buffer + debug_buf_idx, DEBUG_BUF_LEN - debug_buf_idx,
    "[f_min: %lu; f_avg: %lu; f_max: %lu] ",
    msg_min_f_Hz, msg_average_f_Hz, msg_max_f_Hz);
}

// --------------- Debug information functions ---------------
void appendInternalTime_ms(
  char debug_buffer[DEBUG_BUF_LEN],
  size_t& debug_buf_idx
){
  debug_buf_idx += snprintf(debug_buffer + debug_buf_idx, DEBUG_BUF_LEN - debug_buf_idx, 
    "[t: %lu] ", millis());
}

void printDebugInfo(
  char debug_buffer[DEBUG_BUF_LEN],
  size_t& debug_buf_idx
){
  Serial.print(debug_buffer);
  debug_buf_idx = 0;
}

// ================================= ARDUINO RX CONTROL LOOP =================================
void setup(){
  mallocMessageBuffer(&message_buffer);
  pinMode(RXpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RXpin), risingEdgeHandler, RISING);
  Serial.begin(250000);
  delay(500);
}

void loop(){
  if (pulse_detected){
    pulse_detected = false;   // Resets the flag

    uint16_t bit = getBitFromPeriod_us(instant_period_us, f_carrier_Hz, f_deviation_Hz);
    frame_buffer = (frame_buffer << 1) | bit;
    
    if (frame_buffer == start_sync_frame){        // If a message-start sync frame has been detected
      start_sync_frame_detected = true;   // Set start_sync_frame flag.
      end_sync_frame_detected = false;    // Clear end_sync_frame flag.

      resetMessageBuffer(message_buffer, frame_buffer, frame_count, frame_bit_count);
      resetFreqCounters(msg_bit_count, msg_cumulative_f_Hz, msg_average_f_Hz, msg_max_f_Hz, msg_min_f_Hz);
      resetErrorCounters(msg_error_count, msg_data_bit_count, msg_error_rate);

    } else if (frame_buffer == end_sync_frame){   // If a message-end sync frame has been detected
      appendInternalTime_ms(debug_buffer, debug_buf_idx);
      appendMsgFreqStats(debug_buffer, debug_buf_idx, msg_average_f_Hz, msg_min_f_Hz, msg_max_f_Hz);
      appendMsgErrorStats(debug_buffer, debug_buf_idx, msg_error_rate);
      printDebugInfo(debug_buffer, debug_buf_idx);
      printReceivedMessage(message_buffer, frame_count);
      
      start_sync_frame_detected = false;  // Set start_sync_frame flag.
      end_sync_frame_detected = true;     // Clear end_sync_frame flag.

    } else if (start_sync_frame_detected && !end_sync_frame_detected){  // If within a data frame
      frame_bit_count++;
      updateMsgAvgFreq(instant_period_us, msg_bit_count, msg_cumulative_f_Hz, msg_average_f_Hz);
      updateMsgMinMaxFreqs(instant_period_us, msg_max_f_Hz, msg_min_f_Hz);

      if (frame_bit_count == BITS_PER_FRAME){
        if (frameDetect(frame_buffer)){
          updateMessageBuffer(frame_buffer, message_buffer, frame_count, msg_error_count);
          frame_count++;
          updateMsgErrorRate(msg_error_count, msg_data_bit_count, msg_error_rate);
        }
        frame_bit_count = 0;
      }
    }
  }
}
