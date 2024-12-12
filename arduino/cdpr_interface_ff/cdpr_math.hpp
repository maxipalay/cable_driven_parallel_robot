#ifndef CDPR_MATH_HPP

#define CDPR_MATH_HPP

#include "cdpr_globals.hpp"
#include <math.h>

float lengthToTurns(float length, float diameter = 0.08);
float turnsToLength(float turns, float diameter = 0.08);
float tensionToTorque(float tension, float radius = 0.04);
float torqueToTension(float torque, float radius = 0.04);

#endif