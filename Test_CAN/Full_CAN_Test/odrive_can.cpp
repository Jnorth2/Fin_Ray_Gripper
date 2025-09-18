#include "odrive_can.h"
#include <cmath>



// -------------------- Constructor --------------------
OdriveCAN::OdriveCAN(uint8_t node_id)
    : node_id_(node_id), mode_(OdriveMode::IDLE), state_(OdriveState::IDLE) {}


// Call this from your timer or task to read CAN messages
void OdriveCAN::read_can() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) { // non-blocking receive
      parse_can_message(msg);
  }
}


std::vector<twai_message_t> OdriveCAN::get_can_buffer(size_t max_return_msgs) {
    std::vector<twai_message_t> can_msgs;
    can_msgs.reserve(max_return_msgs);

    twai_message_t msg;
    size_t msg_count = 0;

    while (true) {
        // Stop if we've reached the max limit
        if (msg_count >= max_return_msgs) {
            break;
        }

        // Non-blocking read
        if (twai_receive(&msg, 0) != ESP_OK) {
            break;  // No more messages in the buffer
        }

        can_msgs.push_back(msg);
        msg_count++;
    }

    return can_msgs;
}



void OdriveCAN::parse_can_message(const twai_message_t &msg) {
  uint8_t node_mask = (1 << 6) - 1; // 0x3F
  uint8_t cmd_mask = (1 << 5) - 1;  // 0x1F

  uint8_t msg_node_id = (msg.identifier >> 5) & node_mask;
  uint8_t cmd_id     = msg.identifier & cmd_mask;

  if (msg_node_id != node_id_) return; // skip other nodes

  switch (cmd_id) {
      case 0x01: // Heartbeat
          // Optional: handle heartbeat
          break;

      case 0x09: // Encoder Estimate
          if (msg.data_length_code == 8) {
              float pos, vel;
              memcpy(&pos, msg.data, 4);
              memcpy(&vel, msg.data + 4, 4);
              feedback.position = pos;
              feedback.velocity = vel;
          }
          break;

      case 0x14: // Q-axis current
          if (msg.data_length_code == 8) {
              float iq_set, iq_measured;
              memcpy(&iq_set, msg.data, 4);
              memcpy(&iq_measured, msg.data + 4, 4);
              feedback.current = iq_measured;
          }
          break;

      case 0x1C: // Torque Target/Estimate
          if (msg.data_length_code == 8) {
              float torque_set, torque_measured;
              memcpy(&torque_set, msg.data, 4);
              memcpy(&torque_measured, msg.data + 4, 4);
              feedback.torque = torque_measured;
              feedback.torque_setpoint = torque_set;
          }
          break;

      default:
          break;
  }
}

// Wrapper: read and parse everything immediately
void OdriveCAN::read_and_parse_all(size_t max_return_msgs) {
  auto can_msgs = get_can_buffer(max_return_msgs);

  for (auto &msg : can_msgs) {
      parse_can_message(msg);
  }
}

//Convert data to byte form little endian 
template<typename T>
void OdriveCAN::write_le(uint8_t *buf, T value) {
    // Copy raw bytes of value into a temp buffer
    uint8_t tmp[sizeof(T)];
    memcpy(tmp, &value, sizeof(T));

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    // CPU is little endian: copy directly
    for (size_t i = 0; i < sizeof(T); i++) {
        buf[i] = tmp[i];
    }
#else
    // CPU is big endian: reverse order
    for (size_t i = 0; i < sizeof(T); i++) {
        buf[i] = tmp[sizeof(T) - 1 - i];
    }
#endif
}

//Write a can frame to send
twai_message_t OdriveCAN::write_can_frame(uint8_t node_id, uint8_t cmd_id, uint8_t *data, int data_size){
  twai_message_t message;
  message.identifier = (node_id << 5) | cmd_id;
  message.extd = 0;  // standard ID
  message.rtr = 0;
  message.data_length_code = data_size;

  for (size_t i = 0; i < data_size; i++){
    message.data[i] = data[i];
  }
  
  return message;
}

void OdriveCAN::print_can_frame_bits(const twai_message_t &msg) {
  Serial.print("ID: ");
  Serial.print(msg.identifier, HEX);
  Serial.print(" DLC: ");
  Serial.println(msg.data_length_code);

  for (int i = 0; i < msg.data_length_code; i++) {
    uint8_t byte = msg.data[i];
    // Print each bit of this byte
    for (int bit = 7; bit >= 0; bit--) {
      Serial.print((byte >> bit) & 0x01);
    }
    Serial.print(" ");
  }
  Serial.println();
}

void OdriveCAN::print_can_frame_hex(const twai_message_t &msg) {
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


bool OdriveCAN::send_can_frame(twai_message_t &message){
  print_can_frame_hex(message);
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK) {
      Serial.println("Failed to send CAN Frame");
      return false;
  }
  return true;
}



void OdriveCAN::set_mode(OdriveMode mode){
  /*Sets the desired control mode. 
        
  Parameters
  ----------
      mode : OdriveMode
          The desired control mode {idle, closed loop(CL) torque, CL ramped velocity , CL trapezoidal 
          trajectory}
  */
  if(mode != mode_){
    bool success = true;
    Serial.println("Switching Modes");
    twai_message_t message;
    switch (mode){
      case OdriveMode::IDLE:{
        //Idle
        //Axis State
        uint8_t clc_buf[4];
        unsigned int axis_state = 1; //Idle
        write_le(clc_buf, axis_state);
        message = write_can_frame(NODE_ID, 0x07, clc_buf, sizeof(clc_buf));
        success = success ? send_can_frame(message) : false;

        if(!success){
          Serial.println("Failed to switch to Idle Mode");
        }
        mode_ = mode;
        break;
      }
      case OdriveMode::TORQUE:{
        //torq
        //Axis State
        uint8_t clc_buf[4];
        unsigned int axis_state = 8; //Closed Loop control
        write_le(clc_buf, axis_state);
        message = write_can_frame(NODE_ID, 0x07, clc_buf, sizeof(clc_buf));
        success = success ? send_can_frame(message) : false;

        //Set Control Mode
        uint8_t cm_buf[8];
        unsigned int control_mode = 1; //Torque
        unsigned int input_mode = 1;  //Passthrough
        write_le(&cm_buf[0], control_mode);
        write_le(&cm_buf[4], input_mode);
        message = write_can_frame(NODE_ID, 0x0b, cm_buf, sizeof(cm_buf));
        success = success ? send_can_frame(message) : false;

        if(!success){
          Serial.println("Failed to switch to Torque Mode");
        }
        mode_ = mode;
        break;
      }
      case OdriveMode::VELOCITY: {
        //vel
        //Axis State
        uint8_t clc_buf[4];
        unsigned int axis_state = 8; //Closed Loop control
        write_le(clc_buf, axis_state);
        message = write_can_frame(NODE_ID, 0x07, clc_buf, sizeof(clc_buf));
        success = success ? send_can_frame(message) : false;

        //Set Control Mode
        uint8_t cm_buf[8];
        unsigned int control_mode = 2; //Velocity
        unsigned int input_mode = 2; //Vel_ramp
        write_le(&cm_buf[0], control_mode);
        write_le(&cm_buf[4], input_mode);
        message = write_can_frame(NODE_ID, 0x0b, cm_buf, sizeof(cm_buf));
        success = success ? send_can_frame(message) : false;

        //Set Vel Ramp Rate
        uint8_t vrr_buf[8];
        uint8_t op_code = 1;
        uint16_t endpoint = 403;
        uint8_t reserved = 0;
        float vel_ramp_rate = ACCEL_LIMIT;
        write_le(&vrr_buf[0], op_code);
        write_le(&vrr_buf[1], endpoint);
        write_le(&vrr_buf[3], reserved);
        write_le(&vrr_buf[4], vel_ramp_rate);
        message = write_can_frame(NODE_ID, 0x04, vrr_buf, sizeof(vrr_buf));
        success = success ? send_can_frame(message) : false;
        
        if(!success){
          Serial.println("Failed to switch to Velocity Mode");
        }
        mode_ = mode;
        break;
      }
      case OdriveMode::TRAP_TRAJ:{
        //Trap Traj
        //Axis State
        uint8_t clc_buf[4];
        unsigned int axis_state = 8; //Closed Loop control
        write_le(clc_buf, axis_state);
        message = write_can_frame(NODE_ID, 0x07, clc_buf, sizeof(clc_buf));
        success = success ? send_can_frame(message) : false;

        //Set Control Mode
        uint8_t cm_buf[8];
        unsigned int control_mode = 3; //Position
        unsigned int input_mode = 5; //Trap_traj
        write_le(&cm_buf[0], control_mode);
        write_le(&cm_buf[4], input_mode);
        message = write_can_frame(NODE_ID, 0x0b, cm_buf, sizeof(cm_buf));
        success = success ? send_can_frame(message) : false;

        //Set trajectory Vel Limit
        uint8_t vl_buf[4];
        float v_limit = VEL_LIMIT;
        write_le(vl_buf, v_limit);
        message = write_can_frame(NODE_ID, 0x11, vl_buf, sizeof(vl_buf));
        success = success ? send_can_frame(message) : false;

        //Set trajectory accel limit
        uint8_t al_buf[8];
        float a_limit = ACCEL_LIMIT;
        float d_limit = DECCEL_LIMIT;
        write_le(&al_buf[0], a_limit);
        write_le(&al_buf[4], d_limit);
        message = write_can_frame(NODE_ID, 0x12, al_buf, sizeof(al_buf));
        success = success ? send_can_frame(message) : false;

        //Set Limits
        uint8_t lim_buf[8];
        v_limit = VEL_LIMIT;
        float iq_limit = CURRENT_LIMIT;
        write_le(&lim_buf[0], v_limit);
        write_le(&lim_buf[4], iq_limit);
        message = write_can_frame(NODE_ID, 0x0f, lim_buf, sizeof(lim_buf));
        success = success ? send_can_frame(message) : false;

        if(!success){
          Serial.println("Failed to switch to Position Mode");
        }
        mode_ = mode;
        break;
      }
    }
  }
}

void OdriveCAN::homing(){
  state_ = OdriveState::HOMING;
  set_mode(OdriveMode::VELOCITY);
  //Send Velocity Setpoint
  read_and_parse_all(1000);
  velocity_command(CLOSE_DIRECTION * 1);
  Serial.print(feedback.position);
  while (std::abs(feedback.current) < 3){
    read_and_parse_all(1000);
    delay(10);
  }
  set_mode(OdriveMode::IDLE);
  home_pos = feedback.position - CLOSE_DIRECTION * HOME_OFFSET;
  close_pos = feedback.position;
  Serial.print("Home Position: ");
  Serial.println(home_pos);
  Serial.print("Close Position: ");
  Serial.println(close_pos);

  set_mode(OdriveMode::TRAP_TRAJ);
  Serial.println("Sending Home position.");
  position_command(home_pos);
  float pos_error = 0;
  while (true){
    read_and_parse_all(1000);
    pos_error = std::abs(feedback.position - home_pos);
    Serial.print(feedback.position);
    Serial.print(" | ");
    Serial.println(pos_error);
    if (pos_error < 0.015){
      Serial.println("Returned to Home");
      break;
    }
    
    delay(10);
  }
  state_ = OdriveState::OPEN;
  return;
}

void OdriveCAN::close(float pos){
  if (state_ == OdriveState::HOMING){
    return;
  }
  set_mode(OdriveMode::TRAP_TRAJ);
  state_ = OdriveState::CLOSING;
  Serial.println("Closing");
  position_command(pos);
}

void OdriveCAN::open(float pos){
  if (state_ == OdriveState::HOMING){
    return;
  }
  set_mode(OdriveMode::TRAP_TRAJ);
  state_ = OdriveState::OPENING;
  Serial.println("Opening");
  position_command(pos);
}

void OdriveCAN::update_state(){
  read_and_parse_all(1000);
  switch(state_){
    case OdriveState::OPENING:{
      float pos_error = std::abs(feedback.position - home_pos);
      if (pos_error < 0.015){
        state_ = OdriveState::OPEN;
        Serial.println("Open");
      }
      break;
    }
    case OdriveState::CLOSING:{
      float pos_error = std::abs(feedback.position - home_pos);
      if (std::abs(feedback.current) > CURRENT_THRESHOLD){
        torque_command(CLOSE_DIRECTION * TORQUE_SETPOINT);
        state_ = OdriveState::CLOSED;
        Serial.println("Closed with Torque");
      }
      else if (pos_error < 0.015){
        state_ = OdriveState::CLOSED;
        Serial.println("Closed with position");
      }
      break;
    }
  }
}

void OdriveCAN::velocity_command(float vel, float torq_ff){
  /*Send a velocity set point through can.
        
  Parameters
  ----------
      vel : float
          A velocity setpoint.
      torq_ff : float, optional
          The torque feed forward value.
  */
  twai_message_t message;
  uint8_t data_buf[8];
  write_le(&data_buf[0], vel);
  write_le(&data_buf[4], torq_ff);
  message = write_can_frame(node_id_, 0x0d, data_buf, sizeof(data_buf));
  bool success = send_can_frame(message);

  if(!success){
    Serial.println("Failed to send velocity");
  }

}

void OdriveCAN::torque_command(float torq){
  /*Send a torque set point through can.
        
  Parameters
  ----------
      torq : float
          A torque set point. 
  */
  twai_message_t message;
  uint8_t data_buf[4];
  write_le(data_buf, torq);
  message = write_can_frame(node_id_, 0x0e, data_buf, sizeof(data_buf));
  bool success = send_can_frame(message);

  if(!success){
    Serial.println("Failed to send torque");
  }
}

void OdriveCAN::position_command(float pos, int16_t vel_ff, int16_t torq_ff){
  /*Send a position set point through can.
        
  Parameters
  ----------
      pos : float
          A position set point. 
      vel_ff : int, optional
          The velocity feed forward value in 1/1000 rev/s.
      torq_ff : int, optional
          The torque feed forward value in 1/1000 Nm.
  */
  twai_message_t message;
  uint8_t data_buf[8] = {0};
  write_le(&data_buf[0], pos);
  write_le(&data_buf[4], vel_ff);
  write_le(&data_buf[6], torq_ff);
  message = write_can_frame(node_id_, 0x0c, data_buf, sizeof(data_buf));
  bool success = send_can_frame(message);

  if(!success){
    Serial.println("Failed to send position");
  }
}



OdriveMode OdriveCAN::get_mode(){
  return mode_;
}
    

OdriveState OdriveCAN::get_state(){
  return state_;
}
