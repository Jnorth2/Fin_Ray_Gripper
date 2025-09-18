#include <Arduino.h>
#include "driver/twai.h"

void print_can_frame_hex(const twai_message_t &msg) {
  Serial.print("ID: 0x");
  Serial.print(msg.identifier, HEX);
  Serial.print(" DLC: ");
  Serial.print(msg.data_length_code);
  Serial.print(" Data: ");

  for (int i = 0; i < msg.data_length_code; i++) {
    if (msg.data[i] < 0x10) Serial.print("0"); // pad single-digit hex
    Serial.print(msg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

twai_message_t msg;


void setup() {
  Serial.begin(115200);
  delay(500);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_18, GPIO_NUM_19, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  delay(500);
  twai_start();
  delay(500);
  msg.identifier = 0x123;
  msg.extd = 0;
  msg.data_length_code = 1;
  msg.data[0] = 0x42;

  if(twai_transmit(&msg, pdMS_TO_TICKS(1000)) != ESP_OK){
    Serial.println("Failed to send CAN Frame");
  } else {
    Serial.println("CAN Frame sent!");
  }
}

void loop() {
//  if(twai_transmit(&msg, pdMS_TO_TICKS(1000)) != ESP_OK){
//    Serial.println("Failed to send CAN Frame");
//  } else {
//    Serial.println("CAN Frame sent!");
//  }
//  delay(1000);
  twai_message_t msg_rx;

  // Wait for a CAN message
  if (twai_receive(&msg_rx, pdMS_TO_TICKS(1000)) == ESP_OK) {
    print_can_frame_hex(msg_rx);  // Print it in hex
  } else {
    // No message received in 1 second
    Serial.println("No CAN message");
  }
}
