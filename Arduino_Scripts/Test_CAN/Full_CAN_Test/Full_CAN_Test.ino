


#include <driver/twai.h>

#include "driver/gpio.h"
#include "odrive_can.h"


// CAN Setup
#define RX_PIN GPIO_NUM_19
#define TX_PIN GPIO_NUM_18
#define CAN_BAUD 500E3

OdriveCAN odrive(NODE_ID);


void setup() {
  // put your setup code here, to run once:


  Serial.begin(115200);
  delay(500); // Wait for Serial to stabilize  

  delay(2000); // wait for odrive to boot


  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK){
    Serial.println("Failed to install CAN drivers");
  }
  else if (twai_start() != ESP_OK){
    Serial.println("Failed to start CAN driver");
  }
  odrive.set_mode(OdriveMode::VELOCITY);
  odrive.set_mode(OdriveMode::IDLE);
  odrive.set_mode(OdriveMode::TRAP_TRAJ);
  odrive.set_mode(OdriveMode::IDLE);
  Serial.println("Sending Position of 1.0");
  //odrive.position_command(1.0);
  odrive.homing();
}

void loop() {
  odrive.update_state();
 // Example trigger
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'o') odrive.open(odrive.home_pos);
        if (cmd == 'c') odrive.close(odrive.close_pos);
    }

    delay(10);
}
