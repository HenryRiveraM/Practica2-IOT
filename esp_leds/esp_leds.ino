#include "LedExecute.hpp"

Led leds[] = {Led(21), Led(22), Led(23)};

LedExecute application(1, leds, 21, 3, "192.168.1.7", 12345, "RIVERA WIFI 2.4", "2880203CB.");

void setup(){
    application.setup();
}
void loop(){
    application.loop();
}