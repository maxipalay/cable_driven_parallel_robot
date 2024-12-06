#ifndef CDPR_SERIAL_HPP

#define CDPR_SERIAL_HPP

#include "cdpr_globals.hpp"

void processInput(String input);
String formatLengths(CableLengths lengths);
String formatTorques(CableLengths torques);
void serialInputFlush();

#endif