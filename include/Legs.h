#ifndef LEG_H
#define LEG_H

#include "vectors.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

#define SERVOMIN 100 // minimum pulse to move servo
#define SERVOMAX 550 // pulse to get to 180deg

class Leg {
private:
  Adafruit_PWMServoDriver *pwmBoard;
  int pinCoxa, pinFemur, pinTibia;

  float offCoxa, offFemur, offTibia;
  bool invCoxa, invFemur, invTibia;

  const float T = 154.3; // tibia length
  const float F = 110.0; // femur length
  const float H = 31.4;  // coxa length

  unsigned long moveStartTime = 0;
  unsigned long moveDuration = 0;
  Vector3 startPos;
  bool isMoving = false;

  unsigned long lastFrameTime = 0;
  const unsigned long FRAME_INTERVAL = 20;

  int angleToPulse(float ang, bool invert = false) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    if (invert) {
      pulse = map(pulse, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN);
    }
    return pulse;
  }

public:
  Vector3 currentPos = Vector3(100, 0, 0);
  Vector3 targetPos;

  Leg(Adafruit_PWMServoDriver *board, int pC, int pF, int pT, float oC,
      float oF, float oT, bool iC, bool iF, bool iT) {

    pwmBoard = board;
    pinCoxa = pC;
    pinFemur = pF;
    pinTibia = pT;
    offCoxa = oC;
    offFemur = oF;
    offTibia = oT;
    invCoxa = iC;
    invFemur = iF;
    invTibia = iT;
  }

  void setTarget(Vector3 target, unsigned long duration) {
    startPos = currentPos;
    targetPos = target;
    moveStartTime = millis();
    moveDuration = duration;
    isMoving = true;
  }

  // Directly set joint angles (Bypasses IK)
  void setAngles(float coxaAngle, float femurAngle, float tibiaAngle) {
    isMoving = false;

    pwmBoard->setPWM(pinCoxa, 0, angleToPulse(coxaAngle + offCoxa, invCoxa));
    pwmBoard->setPWM(pinFemur, 0,
                     angleToPulse(femurAngle + offFemur, invFemur));
    pwmBoard->setPWM(pinTibia, 0,
                     angleToPulse(tibiaAngle + offTibia, invTibia));
  }

  // Instantly calculate and move to a Cartesian coordinate
  void setInstantIK(Vector3 target) {
    isMoving = false; // Turn off the point-to-point interpolator
    currentPos = target;

    float x = currentPos.x;
    float y = currentPos.y;
    float z = currentPos.z * -1;

    // IK Math
    float a1 = atan(y / (x + H));
    float x_calc = sqrt(pow(x + H, 2) + pow(y, 2)) - H;
    float L = sqrt(x_calc * x_calc + z * z);

    float a2 = acos((F * F + L * L - T * T) / (2 * F * L)) - atan(z / x_calc);
    float a3 = acos((F * F + T * T - L * L) / (2 * F * T));

    a1 = a1 * 57.2958;
    a2 = a2 * 57.2958;
    a3 = a3 * 57.2958;

    // Write directly to servos
    pwmBoard->setPWM(pinCoxa, 0, angleToPulse(a1 + offCoxa, invCoxa));
    pwmBoard->setPWM(pinFemur, 0, angleToPulse(a2 + offFemur, invFemur));
    pwmBoard->setPWM(pinTibia, 0, angleToPulse(a3 + offTibia, invTibia));
  }

  // Calculate IK and write to the servo driver smoothly over time
  void update() {
    if (!isMoving)
      return;

    unsigned long currentMillis = millis();

    if (currentMillis - lastFrameTime < FRAME_INTERVAL)
      return;

    lastFrameTime = currentMillis; // Reset the leg's internal timer

    float progress = (float)(currentMillis - moveStartTime) / moveDuration;

    if (progress >= 1.0) {
      progress = 1.0;
      isMoving = false;
    }

    float currentX = startPos.x + (targetPos.x - startPos.x) * progress;
    float currentY = startPos.y + (targetPos.y - startPos.y) * progress;
    float currentZ = startPos.z + (targetPos.z - startPos.z) * progress;

    currentPos = Vector3(currentX, currentY, currentZ);

    float x = currentPos.x;
    float y = currentPos.y;
    float z = -1.0 * currentPos.z;

    float a1 = atan(y / (x + H));
    float x_calc = sqrt(pow(x + H, 2) + pow(y, 2)) - H;
    float L = sqrt(x_calc * x_calc + z * z);

    float a2 = acos((F * F + L * L - T * T) / (2 * F * L)) - atan(z / x_calc);
    float a3 = acos((F * F + T * T - L * L) / (2 * F * T));

    a1 = a1 * 57.2958;
    a2 = a2 * 57.2958;
    a3 = a3 * 57.2958;

    pwmBoard->setPWM(pinCoxa, 0, angleToPulse(a1 + offCoxa, invCoxa));
    pwmBoard->setPWM(pinFemur, 0, angleToPulse(a2 + offFemur, invFemur));
    pwmBoard->setPWM(pinTibia, 0, angleToPulse(a3 + offTibia, invTibia));
  }
};

#endif
