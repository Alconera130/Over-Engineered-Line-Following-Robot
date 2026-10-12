#include <Arduino.h>

#include "Robot.h"
Robot robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
