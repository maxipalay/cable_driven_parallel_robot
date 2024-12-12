#include "cdpr_serial.hpp"

void processInput(String input) {
  input.trim();  // Remove any leading/trailing whitespace

  // Split the input string by spaces
  int firstSpaceIndex = input.indexOf(' ');
  if (firstSpaceIndex != -1) {
    char firstLetter = input.charAt(0);  // Get the first letter
    // Serial.print("First letter: ");
    // Serial.println(firstLetter);
    // Extract the substring starting from the first space
    String numbersString = input.substring(firstSpaceIndex + 1);

    // Split numbers by spaces
    int lastIndex = 0;
    int nextIndex;
    int counter = 0;

    // Serial.println("Numbers:");
    while ((nextIndex = numbersString.indexOf(' ', lastIndex)) != -1) {
      String number = numbersString.substring(lastIndex, nextIndex);
      switch (counter) {
        case 0:
          cable_lengths.l1 = number.toFloat();
          break;
        case 1:
          cable_lengths.l2 = number.toFloat();
          break;
        case 2:
          cable_lengths.l3 = number.toFloat();
          break;
        case 3:
          cable_lengths.l4 = number.toFloat();
          break;
        case 4:
          cable_lengths.l5 = number.toFloat();
          break;
        case 5:
          cable_lengths.l6 = number.toFloat();
          break;
        case 6:
          cable_lengths.l7 = number.toFloat();
          break;
        case 7:
          cable_lengths.l8 = number.toFloat();
          break;
        case 8:
          cable_tensions.l1 = number.toFloat();
          break;
        case 9:
          cable_tensions.l2 = number.toFloat();
          break;
        case 10:
          cable_tensions.l3 = number.toFloat();
          break;
        case 11:
          cable_tensions.l4 = number.toFloat();
          break;
        case 12:
          cable_tensions.l5 = number.toFloat();
          break;
        case 13:
          cable_tensions.l6 = number.toFloat();
          break;
        case 14:
          cable_tensions.l7 = number.toFloat();
          break;
      }
      // Serial.println(number);
      lastIndex = nextIndex + 1;
      counter++;
    }
    // Print the last number
    // Serial.println(numbersString.substring(lastIndex));
    cable_tensions.l8 = numbersString.substring(lastIndex).toFloat();

  } else {
    // Serial.println("Invalid input format.");
  }
}

String formatLengths(CableLengths lengths) {
  String result = "fp" + String(lengths.l1, 4) + "," + String(lengths.l2, 4) + "," + String(lengths.l3, 4) + "," + String(lengths.l4, 4) + "," + String(lengths.l5, 4) + "," + String(lengths.l6, 4) + "," + String(lengths.l7, 4) + "," + String(lengths.l8, 4);
  return result;
}

String formatTorques(CableLengths torques) {
  String result = "ft" + String(torques.l1, 4) + "," + String(torques.l2, 4) + "," + String(torques.l3, 4) + "," + String(torques.l4, 4) + "," + String(torques.l5, 4) + "," + String(torques.l6, 4) + "," + String(torques.l7, 4) + "," + String(torques.l8, 4);
  return result;
}

void serialInputFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}