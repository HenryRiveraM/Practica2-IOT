#include "SensorExecute.hpp"
const uint16_t SENSOR_ID = 1;

SensorExecute aplication(SENSOR_ID, 2, 4, "192.168.1.7", 12345, "RIVERA WIFI 2.4", "2880203CB.");
void setup() {
  aplication.setup();
} 

void loop() {
  aplication.loop();
}