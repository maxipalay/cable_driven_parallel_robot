#ifndef CDPR_CAN_HPP

#define CDPR_CAN_HPP

#include "cdpr_globals.hpp"
// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html


void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void onFeedbackTorques(Get_Torques_msg_t& msg, void* user_data);
void onCanMessage(const CanMsg& msg);
bool setupCan();

#endif
