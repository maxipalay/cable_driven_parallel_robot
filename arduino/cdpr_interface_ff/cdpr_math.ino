#include "cdpr_math.hpp"


float lengthToTurns(float length, float diameter) {
  float turns = length / (PI * diameter);  // Calculate the number of turns
  return turns;
}

float turnsToLength(float turns, float diameter) {
  float length = turns * (PI * diameter);  // Calculate the length from turns
  return length;
}

float tensionToTorque(float tension, float radius) {
  /*
    Convert tension (in newtons) to torque (in newton-meters).
    
    Parameters:
    - tension: The force in newtons.
    - radius: The distance from the center of rotation to the point of application (default: 0.04 meters).
    
    Returns:
    - The calculated torque in newton-meters.
    */
  float res = -tension * radius;
  if (res > -0.1) {
    return -0.1;
  }
  if (res < -2.0) {
    return -2.0;
  }

  return res;
}

float torqueToTension(float torque, float radius){
  return -torque/radius;
}