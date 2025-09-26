#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <Arduino.h>
#include "driver/twai.h"
#include <vector>

// -------------------- Constants --------------------
#define NODE_ID 6 // ODrive node ID
#define VEL_LIMIT 35 //[rev/s]
#define ACCEL_LIMIT 80 //[rev/s^2]
#define DECCEL_LIMIT 80 //[rev/s^2]
#define CLOSE_DIRECTION -1 //[bool]
#define HOME_OFFSET 14 //[rev]
#define VEL_SETPOINT 25 //[rev/s]
#define CURRENT_THRESHOLD 3 //[A]
#define TORQUE_SETPOINT 0.25 //[Nm]
#define CURRENT_LIMIT 4 //[A]

// -------------------- Enums --------------------
enum class OdriveMode {
    IDLE,
    TORQUE,
    VELOCITY,
    TRAP_TRAJ
};

enum class OdriveState {
    HOMING,
    IDLE,
    OPEN,
    CLOSED,
    CLOSING,
    OPENING
};

// -------------------- OdriveCAN Class --------------------
class OdriveCAN {
public:
    // Constructor
    OdriveCAN(uint8_t node_id);

    // CAN frame helpers
    twai_message_t create_can_frame(uint8_t cmd_id, uint8_t *data, int data_size);
    twai_message_t write_can_frame(uint8_t node_id, uint8_t cmd_id, uint8_t *data, int data_size);
    bool send_can_frame(twai_message_t &msg);
    void print_can_frame_bits(const twai_message_t &msg);
    void print_can_frame_hex(const twai_message_t &msg);

    // ODrive control functions
    void set_mode(OdriveMode mode);
    void homing();
    void close(float pos);
    void open(float pos);
    void update_state();

    // Commands
    void velocity_command(float vel, float torq_ff = 0.0f);
    void torque_command(float torq);
    void position_command(float pos, int16_t vel_ff = 0, int16_t torq_ff = 0);

    // CAN feedback
    struct Feedback {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;
        float current = 0.0f;
        float torque_setpoint = 0.0f;
    };

    Feedback feedback;
    float home_pos = 0;
    float close_pos = 0;
    // Read and parse incoming CAN frames
    void read_can();                   // Call this periodically
    void parse_can_message(const twai_message_t &msg);
    void read_and_parse_all(size_t max_return_msgs = 1000);
    std::vector<twai_message_t> get_can_buffer(size_t max_return_msgs = 1000);

    //Getters
    OdriveMode get_mode();
    OdriveState get_state();

private:
    uint8_t node_id_;
    OdriveMode mode_;
    OdriveState state_;

    // Helper to convert data to little-endian byte array
    template<typename T>
    void write_le(uint8_t *buf, T value);
};


#endif // ODRIVE_CAN_H
