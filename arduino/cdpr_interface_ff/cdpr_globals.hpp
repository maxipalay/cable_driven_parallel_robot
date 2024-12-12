
#ifndef CDPR_GLOBALS_HPP

#define CDPR_GLOBALS_HPP

#include <Arduino.h>
#include "ODriveCAN.h"
#include "TeensyTimerTool.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

#define CAN_BAUDRATE 500000
#define DEBUG true
// ODrive node_id for odrv1
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3
#define ODRV4_NODE_ID 4
#define ODRV5_NODE_ID 5
#define ODRV6_NODE_ID 6
#define ODRV7_NODE_ID 7
#define ODRV8_NODE_ID 8

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  Get_Torques_msg_t last_torques_feedback;
  bool received_feedback = false;
  float offset = 0.0;
};

struct CableLengths {
  float l1;
  float l2;
  float l3;
  float l4;
  float l5;
  float l6;
  float l7;
  float l8;
};

// cable lengths requested via serial
extern CableLengths cable_lengths;
extern CableLengths cable_tensions;

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
 // Instantiate ODrive objects
extern ODriveCAN odrv1;                                    // Standard CAN message ID
extern ODriveCAN odrv2;                                    // Standard CAN message ID
extern ODriveCAN odrv3;                                    // Standard CAN message ID
extern ODriveCAN odrv4;                                    // Standard CAN message ID
extern ODriveCAN odrv5;                                    // Standard CAN message ID
extern ODriveCAN odrv6;                                    // Standard CAN message ID
extern ODriveCAN odrv7;                                    // Standard CAN message ID
extern ODriveCAN odrv8;                                    // Standard CAN message ID
extern ODriveCAN* odrives[8];
 // Keep some application-specific user data for every ODrive.
extern ODriveUserData odrv1_user_data;
extern ODriveUserData odrv2_user_data;
extern ODriveUserData odrv3_user_data;
extern ODriveUserData odrv4_user_data;
extern ODriveUserData odrv5_user_data;
extern ODriveUserData odrv6_user_data;
extern ODriveUserData odrv7_user_data;
extern ODriveUserData odrv8_user_data;
extern void* odrives_user_data[8];

extern String feedback_pos;
extern String feedback_torques;
extern volatile bool feedback_pos_time;
extern volatile bool motor_command_time;
extern volatile int point_index;
extern volatile bool send_points;

extern float feedback1_cpy;
extern float feedback2_cpy;
extern float feedback3_cpy;
extern float feedback4_cpy;
extern float feedback5_cpy;
extern float feedback6_cpy;
extern float feedback7_cpy;
extern float feedback8_cpy;

extern float feedback1_torque_cpy;
extern float feedback2_torque_cpy;
extern float feedback3_torque_cpy;
extern float feedback4_torque_cpy;
extern float feedback5_torque_cpy;
extern float feedback6_torque_cpy;
extern float feedback7_torque_cpy;
extern float feedback8_torque_cpy;

extern float position1;
extern float position2;
extern float position3;
extern float position4;
extern float position5;
extern float position6;
extern float position7;
extern float position8;

extern float torque1;
extern float torque2;
extern float torque3;
extern float torque4;
extern float torque5;
extern float torque6;
extern float torque7;
extern float torque8;

extern bool update_positions;

#endif