#include <Arduino.h>
#define TRIG 27
#define ECHO 21

void setup()
{
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
}
void loop()
{
    // Standard HC-SR04 trigger pulse: low -> 10us high -> low
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long delta_time = pulseIn(ECHO, HIGH, 30000);
    float detect_distance = delta_time * 0.0343f / 2.0f;
    Serial.printf("distance=%.2f cm (echo_us=%lu)\n", detect_distance, delta_time);
    delay(500);

}
