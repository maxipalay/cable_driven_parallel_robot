#include "cdpr_globals.hpp"
#include "cdpr_serial.hpp"
#include "cdpr_can.hpp"
#include "cdpr_math.hpp"

struct ODriveStatus;  // hack to prevent teensy compile error
struct CableLengths;

// cable lengths requested via serial
CableLengths cable_lengths = { 0.769, 0.769, 0.769, 0.769, 0.769, 0.769, 0.769, 0.769 };
CableLengths cable_tensions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// Instantiate ODrive objects
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv4(wrap_can_intf(can_intf), ODRV4_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv5(wrap_can_intf(can_intf), ODRV5_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv6(wrap_can_intf(can_intf), ODRV6_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv7(wrap_can_intf(can_intf), ODRV7_NODE_ID);                                    // Standard CAN message ID
ODriveCAN odrv8(wrap_can_intf(can_intf), ODRV8_NODE_ID);                                    // Standard CAN message ID
ODriveCAN* odrives[] = { &odrv1, &odrv2, &odrv3, &odrv4, &odrv5, &odrv6, &odrv7, &odrv8 };  // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
ODriveUserData odrv4_user_data;
ODriveUserData odrv5_user_data;
ODriveUserData odrv6_user_data;
ODriveUserData odrv7_user_data;
ODriveUserData odrv8_user_data;
void* odrives_user_data[] = { &odrv1_user_data, &odrv2_user_data, &odrv3_user_data, &odrv4_user_data, &odrv5_user_data, &odrv6_user_data, &odrv7_user_data, &odrv8_user_data };  // Make sure all ODriveCAN instances are accounted for here

String feedback_pos;
String feedback_torques;
volatile bool feedback_pos_time = false;
volatile bool motor_command_time = false;
volatile int point_index = 0;
volatile bool send_points = false;

float feedback1_cpy;
float feedback2_cpy;
float feedback3_cpy;
float feedback4_cpy;
float feedback5_cpy;
float feedback6_cpy;
float feedback7_cpy;
float feedback8_cpy;

float feedback1_torque_cpy;
float feedback2_torque_cpy;
float feedback3_torque_cpy;
float feedback4_torque_cpy;
float feedback5_torque_cpy;
float feedback6_torque_cpy;
float feedback7_torque_cpy;
float feedback8_torque_cpy;


float position1 = 0.0;
float position2 = 0.0;
float position3 = 0.0;
float position4 = 0.0;
float position5 = 0.0;
float position6 = 0.0;
float position7 = 0.0;
float position8 = 0.0;

float torque1 = 0.0;
float torque2 = 0.0;
float torque3 = 0.0;
float torque4 = 0.0;
float torque5 = 0.0;
float torque6 = 0.0;
float torque7 = 0.0;
float torque8 = 0.0;

bool update_positions = true;

TeensyTimerTool::PeriodicTimer timer_fb;  // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
TeensyTimerTool::PeriodicTimer timer_ct;  // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)

void setup() {

  // enable serial communication
  Serial.begin(500000);

  // register callbacks for incoming CAN messages
  registerOdriveCallbacks();

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ;  // spin indefinitely
  }

  // wait until we get heartbeat from every odrive
  waitForOdrives();

  // enable closed loop control for all Odrives
  enableOdrivesClosedLoopControl();

  odrv1.setLimits(3.0, 25.0);
  delay(10);
  odrv2.setLimits(3.0, 25.0);
  delay(10);
  odrv3.setLimits(3.0, 25.0);
  delay(10);
  odrv4.setLimits(3.0, 25.0);
  delay(10);
  odrv5.setLimits(3.0, 25.0);
  delay(10);
  odrv6.setLimits(3.0, 25.0);
  delay(10);
  odrv7.setLimits(3.0, 25.0);
  delay(10);
  odrv8.setLimits(3.0, 25.0);
  delay(10);
  // homing procedure
  homing();

  odrv1.setLimits(5.0, 20.0);
  delay(10);
  odrv2.setLimits(5.0, 20.0);
  delay(10);
  odrv3.setLimits(5.0, 20.0);
  delay(10);
  odrv4.setLimits(5.0, 20.0);
  delay(10);
  odrv5.setLimits(5.0, 20.0);
  delay(10);
  odrv6.setLimits(5.0, 20.0);
  delay(10);
  odrv7.setLimits(5.0, 20.0);
  delay(10);
  odrv8.setLimits(5.0, 20.0);
  delay(10);

  // set all controllers to position mode
  odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv4.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv5.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv6.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv7.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv8.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);


  timer_fb.begin(copyFeedbackInterrupt, 20'000);
  timer_ct.begin(motorCommandInterrupt, 20'000);

  serialInputFlush();
}

void copyFeedbackInterrupt(void) {
  feedback_pos_time = true;
}

void motorCommandInterrupt(void) {
  motor_command_time = true;
}

void loop() {
  pumpEvents(can_intf);  // This is required on some platforms to handle incoming feedback CAN messages

  if (feedback_pos_time) {
    noInterrupts();
    feedback_pos_time = false;
    feedback1_cpy = odrv1_user_data.last_feedback.Pos_Estimate;
    feedback2_cpy = odrv2_user_data.last_feedback.Pos_Estimate;
    feedback3_cpy = odrv3_user_data.last_feedback.Pos_Estimate;
    feedback4_cpy = odrv4_user_data.last_feedback.Pos_Estimate;
    feedback5_cpy = odrv5_user_data.last_feedback.Pos_Estimate;
    feedback6_cpy = odrv6_user_data.last_feedback.Pos_Estimate;
    feedback7_cpy = odrv7_user_data.last_feedback.Pos_Estimate;
    feedback8_cpy = odrv8_user_data.last_feedback.Pos_Estimate;
    feedback1_torque_cpy = odrv1_user_data.last_torques_feedback.Torque_Estimate;
    feedback2_torque_cpy = odrv2_user_data.last_torques_feedback.Torque_Estimate;
    feedback3_torque_cpy = odrv3_user_data.last_torques_feedback.Torque_Estimate;
    feedback4_torque_cpy = odrv4_user_data.last_torques_feedback.Torque_Estimate;
    feedback5_torque_cpy = odrv5_user_data.last_torques_feedback.Torque_Estimate;
    feedback6_torque_cpy = odrv6_user_data.last_torques_feedback.Torque_Estimate;
    feedback7_torque_cpy = odrv7_user_data.last_torques_feedback.Torque_Estimate;
    feedback8_torque_cpy = odrv8_user_data.last_torques_feedback.Torque_Estimate;
    interrupts();
    CableLengths lengths = {
      turnsToLength(feedback1_cpy - odrv1_user_data.offset),
      turnsToLength(feedback2_cpy - odrv2_user_data.offset),
      turnsToLength(feedback3_cpy - odrv3_user_data.offset),
      turnsToLength(feedback4_cpy - odrv4_user_data.offset),
      turnsToLength(feedback5_cpy - odrv5_user_data.offset),
      turnsToLength(feedback6_cpy - odrv6_user_data.offset),
      turnsToLength(feedback7_cpy - odrv7_user_data.offset),
      turnsToLength(feedback8_cpy - odrv8_user_data.offset)
    };
    CableLengths tensions = {
      torqueToTension(feedback1_torque_cpy),
      torqueToTension(feedback2_torque_cpy),
      torqueToTension(feedback3_torque_cpy),
      torqueToTension(feedback4_torque_cpy),
      torqueToTension(feedback5_torque_cpy),
      torqueToTension(feedback6_torque_cpy),
      torqueToTension(feedback7_torque_cpy),
      torqueToTension(feedback8_torque_cpy)
    };
    feedback_pos = formatLengths(lengths);
    feedback_torques = formatTorques(tensions);
    Serial.println(feedback_pos);
    Serial.println(feedback_torques);
    Serial.flush();
  }


  String inputString = "";
  if (Serial.available()) {

    // Read the input until a newline
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') break;  // Stop at newline
      inputString += c;
    }
    processInput(inputString);
    update_positions = true;
  }
  if (update_positions) {
    update_positions = false;
    position1 = lengthToTurns(cable_lengths.l1) + odrv1_user_data.offset;
    position2 = lengthToTurns(cable_lengths.l2) + odrv2_user_data.offset;
    position3 = lengthToTurns(cable_lengths.l3) + odrv3_user_data.offset;
    position4 = lengthToTurns(cable_lengths.l4) + odrv4_user_data.offset;
    position5 = lengthToTurns(cable_lengths.l5) + odrv5_user_data.offset;
    position6 = lengthToTurns(cable_lengths.l6) + odrv6_user_data.offset;
    position7 = lengthToTurns(cable_lengths.l7) + odrv7_user_data.offset;
    position8 = lengthToTurns(cable_lengths.l8) + odrv8_user_data.offset;

    torque1 = tensionToTorque(cable_tensions.l1);
    torque2 = tensionToTorque(cable_tensions.l2);
    torque3 = tensionToTorque(cable_tensions.l3);
    torque4 = tensionToTorque(cable_tensions.l4);
    torque5 = tensionToTorque(cable_tensions.l5);
    torque6 = tensionToTorque(cable_tensions.l6);
    torque7 = tensionToTorque(cable_tensions.l7);
    torque8 = tensionToTorque(cable_tensions.l8);
    motor_command_time = true;
  }

  if (motor_command_time) {
    motor_command_time = false;
    odrv1.setPosition(position1, 0.0, torque1);
    odrv2.setPosition(position2, 0.0, torque2);
    odrv3.setPosition(position3, 0.0, torque3);
    odrv4.setPosition(position4, 0.0, torque4);
    odrv5.setPosition(position5, 0.0, torque5);
    odrv6.setPosition(position6, 0.0, torque6);
    odrv7.setPosition(position7, 0.0, torque7);
    odrv8.setPosition(position8, 0.0, torque8);
  }
}



// Register callbacks for the heartbeat and encoder feedback messages
void registerOdriveCallbacks() {
  // Loop through all ODrive objects and register the callbacks
  for (int i = 0; i < 8; i++) {
    odrives[i]->onFeedback(onFeedback, odrives_user_data[i]);
    odrives[i]->onStatus(onHeartbeat, odrives_user_data[i]);
    odrives[i]->onTorques(onFeedbackTorques, odrives_user_data[i]);
  }
}

// Wait until heartbeat from all ODrives has been received
void waitForOdrives() {
  for (int i = 0; i < 8; i++) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(odrives_user_data[i]);
    if (DEBUG) {
      Serial.print("Waiting for ODrive ");
      Serial.println(i);
    }
    while (!odrv_user_data->received_heartbeat) {
      pumpEvents(can_intf);
      delay(10);
    }
  }
}

// Enable closed loop control for all ODrives
void enableOdrivesClosedLoopControl() {
  for (int i = 0; i < 8; i++) {
    // Loop through all ODrive objects, set them to closed loop control
    // and wait for confirmation
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(odrives_user_data[i]);
    while (odrv_user_data->last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      odrives[i]->clearErrors();
      delay(1);
      odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can_intf);
      }
    }
  }
}

// Homing routine for the CDPR
void homing() {
  // The strategy for homing is to set all motors to a minimum tension,
  // and one by one switch them to velocity control, have the end effector hit the pulley,
  // and when the ee has been still for a while then we assume that's the offset

  for (int i = 0; i < 8; i++) {
    odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  }

  int stall = 0;

  for (int i = 0; i < 8; i++) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(odrives_user_data[i]);
    if (DEBUG) {
      Serial.print("Homing ODrive ");
      Serial.println(i);
    }

    for (int i = 0; i < 8; i++) {
      odrives[i]->setTorque(-0.3);
      delay(50);
      odrives[i]->setTorque(-0.15);
    }

    // home axis
    odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrives[i]->setVelocity(-1.0, -0.35);

    while (true) {
      delay(1);
      pumpEvents(can_intf);  // This is required on some platforms to handle incoming feedback CAN messages
      if (odrv_user_data->received_feedback) {
        Get_Encoder_Estimates_msg_t feedback = odrv_user_data->last_feedback;
        odrv_user_data->received_feedback = false;
        //Serial.println(feedback.Vel_Estimate);
        if (feedback.Vel_Estimate < 0.1 && feedback.Vel_Estimate > -0.1) {
          stall++;
        } else {
          stall = 0;
        }
        if (stall > 100) {
          odrv_user_data->offset = feedback.Pos_Estimate;
          break;
        }
      }
    }
    odrives[i]->setTorque(-2.0);
    odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrives[i]->setTorque(-2.0);
    delay(200);
    odrives[i]->setTorque(-1.5);
    delay(200);
    odrives[i]->setTorque(-1.0);
    delay(200);
    odrives[i]->setTorque(-0.5);
    delay(200);
    odrives[i]->setTorque(-0.2);
    delay(200);

    stall = 0;  // reset the stall variable
  }

  for (int i = 0; i < 8; i++) {
    odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrives[i]->setTorque(-0.15);
  }
  delay(2000);
}
