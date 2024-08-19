#include <Arduino.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <protothreads.h>

#define TRIGGER_PIN  52
#define ECHO_PIN     53
#define TRIGGER_PIN_BACKWARD  11
#define ECHO_PIN_BACKWARD     12
#define MAX_DISTANCE 100
#define MIN_DISTANCE_FORWARD 25
#define MAX_SPEED 150
#define MIN_SPEED 1

#define DEBUG_DISTANCE
#undef EMULATION
AF_DCMotor motor1(1);
AF_DCMotor motor2(3);
NewPing sonarForward(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

volatile unsigned int distance = 0;
static pt pt1, pt2, pt3;

#ifdef DEBUG_DISTANCE
void printDistance(uint8_t distance) {
    Serial.write("distance:");
    Serial.print(sonarForward.ping_cm());
    Serial.println();
}
void printText(const char* text) {
    Serial.print(text);
    Serial.println();
}
#endif

static PT_THREAD(read_distance(struct pt *pt)) {
    PT_BEGIN(pt);

    while(1) {
        int currentDistance = sonarForward.ping_cm();
        if(currentDistance == 0) {
            currentDistance = 100;
        }
#ifdef DEBUG_DISTANCE
        printDistance(currentDistance);
#endif


        distance = currentDistance;
        PT_SLEEP(pt, 100);
    }

    PT_END(pt);
}
static PT_THREAD(move_forward(struct pt *pt)) {
    PT_BEGIN(pt);

    while(1) {
        PT_WAIT_UNTIL(pt, distance > MIN_DISTANCE_FORWARD);
        printText("forward");
#ifndef EMULATION
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        motor1.setSpeed(200);
        motor2.setSpeed(200);
#endif
        PT_SLEEP(pt, 100);
    }

    PT_END(pt);
}

static PT_THREAD(try_rotate(struct pt *pt)) {
    PT_BEGIN(pt);

    while(1) {
        PT_WAIT_UNTIL(pt, distance < MIN_DISTANCE_FORWARD);
        printText("rotate");
#ifndef EMULATION
        motor1.run(BACKWARD);
        motor2.run(FORWARD);
        motor1.setSpeed(150);
        motor2.setSpeed(150);
#endif
        PT_SLEEP(pt, 100);
    }

    PT_END(pt);
}

void setup()
{
#ifdef DEBUG_DISTANCE
       Serial.begin(9600);
#endif
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    PT_INIT(&pt3);
}

void loop()
{
    read_distance(&pt1);
    move_forward(&pt2);
    try_rotate(&pt3);
}
