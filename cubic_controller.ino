#include <Arduino.h>
#include "cubic.ver2.0.h"
#include "PID.h"

void setup()
{
    Cubic::begin();
    Serial.begin(115200);
}

void loop()
{

    delay(1);
}