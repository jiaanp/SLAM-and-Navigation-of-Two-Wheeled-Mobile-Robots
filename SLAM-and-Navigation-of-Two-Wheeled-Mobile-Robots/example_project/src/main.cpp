#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.printf("hello world!\n");
    delay(1000);
}