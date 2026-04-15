#include "robot_app.h"

namespace {

robot::RobotApp app;

}  // namespace

void setup() {
  app.setup();
}

void loop() {
  app.loop();
}
