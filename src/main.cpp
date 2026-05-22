#include "Adafruit_PWMServoDriver.h"
#include "Legs.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
Adafruit_PWMServoDriver sb2 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver sb1 = Adafruit_PWMServoDriver(0x41); // bridge A0

// for serial read
int receivedInt = 0;
int firstValue = 0;
int secondValue = 0;
int thirdValue = 0;
bool firstReceived = false;
bool secondReceived = false;

// leg dimentions
float tibia = 186.44;
float femur = 110.0;
float coxa = 31.0;

int mdelay = 5;

float globalWalkAngle = 0.0;
float globalStrideLength = 0.0;
float globalRotationSpeed = 0.0;

float globalPitch = 0.0; // Degrees: Positive = Nose Up, Negative = Nose Down
float globalRoll = 0.0;  // Degrees: Positive = Tilt Right, Negative = Tilt Left
float globalRideHeight = 0.0; // mm: Shift the whole body up or down
                              //
// Positive Y is forward, Negative Y is backward. Positive X is right, Negative
// X is left.
float bodyOffsetY[6] = {121.0, 0.0, -121.0, 121.0, 0.0, -121.0};
float bodyOffsetX[6] = {80.7, 97.3, 80.7, -80.7, -97.3, -80.7};
Leg *legs[6];

Vector3 generateTrajectory(float phase, float defaultX, float standingHeight,
                           float stepLength, float stepHeight, float yOffset) {
  // Ensure phase stays strictly wrapped between 0.0 and 1.0
  phase = phase - floor(phase);

  Vector3 targetPos;

  // NEW: Shift the entire stride forward or backward along the Y-axis
  float yLiftoff = (-stepLength / 2.0) + yOffset;  // Back of the stride
  float yTouchdown = (stepLength / 2.0) + yOffset; // Front of the stride
  float zGround = -standingHeight;

  // ---------------------------------------------------------
  // SWING PHASE: (0.0 to 0.5) - Foot in the air moving forward
  // ---------------------------------------------------------
  if (phase < 0.5) {
    float t = phase * 2.0;

    // Kinematic smoothing
    float easedT = (1.0 - cos(t * PI)) / 2.0;

    // Geometric control points
    Vector3 P0(defaultX, yLiftoff, zGround);
    Vector3 P3(defaultX, yTouchdown, zGround);

    float zControl = zGround + (stepHeight * 1.333);
    Vector3 P1(defaultX, yLiftoff, zControl);
    Vector3 P2(defaultX, yTouchdown, zControl);

    // Bezier curve calculation
    float invT = 1.0 - easedT;
    targetPos = P0 * (invT * invT * invT) + P1 * (3.0 * invT * invT * easedT) +
                P2 * (3.0 * invT * easedT * easedT) +
                P3 * (easedT * easedT * easedT);
  }
  // ---------------------------------------------------------
  // STANCE PHASE: (0.5 to 1.0) - Foot on the ground pushing back
  // ---------------------------------------------------------
  else {
    float t = (phase - 0.5) * 2.0;

    float easedT = (1.0 - cos(t * PI)) / 2.0;

    float currentY = yTouchdown + (yLiftoff - yTouchdown) * easedT;
    targetPos = Vector3(defaultX, currentY, zGround);
  }

  return targetPos;
}

void setup() {
  Serial.begin(115200);
  sb1.begin();
  sb2.begin();
  sb1.setPWMFreq(60);
  sb2.setPWMFreq(60);

  // Right Side (Board 1)
  legs[0] = new Leg(&sb1, 0, 1, 3, 95.0, 110.0 + 23, 22.0 - 20 + 11.3 - 90,
                    true, false, true); // Leg 1 (Top-Right)
  legs[1] = new Leg(&sb1, 4, 5, 6, 90.0 - 10, 75.0 + 20, 7.0 - 19 + 11.3 - 90,
                    true, false, true); // Leg 2 (Mid-Right)
  legs[2] = new Leg(&sb1, 8, 9, 11, 135.0 - 10, 85 + 15, 45.0 - 19 + 11.3 - 90,
                    true, false, true); // Leg 3 (Bottom-Right)

  // Left Side (Board 2)
  legs[3] = new Leg(&sb2, 0, 1, 3, 110.0, 110.0, 13.0 + 11.3 - 90, false, false,
                    true); // Leg 4 (Top-Left)
  legs[4] = new Leg(&sb2, 4, 5, 7, 80.0, 65.0, 17 + 11.3 - 90, false, false,
                    true); // Leg 5 (Mid-Left)
  legs[5] = new Leg(&sb2, 8, 9, 11, 82.0 - 5, 60.0 + 5, 32.0 + 11.3 - 90, false,
                    false, true); // Leg 6 (Bottom-Left)
  Serial.println("For Angles:    A,Leg,Coxa,Femur,Tibia  (e.g., A,0,90,90,90)");
  Serial.println(
      "For Cartesian: C,Leg,X,Y,Z             (e.g., C,0,150,0,-50)");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    char cmdType;
    int legIdx;
    float v1, v2, v3;

    if (sscanf(input.c_str(), "%c,%d,%f,%f,%f", &cmdType, &legIdx, &v1, &v2,
               &v3) == 5) {

      if (legIdx < 0 || legIdx > 5) {
        Serial.println("Error: Leg index must be between 0 and 5.");
        return;
      }

      // 'A' or 'a' for direct Angle control
      if (cmdType == 'A' || cmdType == 'a') {
        legs[legIdx]->setAngles(v1, v2, v3);
        Serial.print("Leg ");
        Serial.print(legIdx);
        Serial.print(" ANGLES set to -> Coxa: ");
        Serial.print(v1);
        Serial.print(" | Femur: ");
        Serial.print(v2);
        Serial.print(" | Tibia: ");
        Serial.println(v3);
      }
      // 'C' or 'c' for Cartesian IK control
      else if (cmdType == 'C' || cmdType == 'c') {
        legs[legIdx]->setTarget(Vector3(v1, v2, v3), 1000);
        Serial.print("Leg ");
        Serial.print(legIdx);
        Serial.print(" TARGET set to -> X: ");
        Serial.print(v1);
        Serial.print(" | Y: ");
        Serial.print(v2);
        Serial.print(" | Z: ");
        Serial.println(v3);
      }
      // 'V' or 'v' for Velocity Control (Omnidirectional + Rotation)
      else if (cmdType == 'V' || cmdType == 'v') {
        globalWalkAngle = v1;
        globalStrideLength = v2;
        globalRotationSpeed = v3;

        Serial.print("ROBOT VELOCITY -> Angle: ");
        Serial.print(globalWalkAngle);
        Serial.print(" deg | Speed: ");
        Serial.print(globalStrideLength);
        Serial.print(" | Rotation: ");
        Serial.println(globalRotationSpeed);
      }
      // 'P' or 'p' for Posture Control (Pitch, Roll, Ride Height)
      else if (cmdType == 'P' || cmdType == 'p') {
        globalPitch = v1;
        globalRoll = v2;
        globalRideHeight = v3;

        Serial.print("ROBOT POSTURE -> Pitch: ");
        Serial.print(globalPitch);
        Serial.print(" deg | Roll: ");
        Serial.print(globalRoll);
        Serial.print(" deg | Ride Height: ");
        Serial.println(globalRideHeight);
      } else {
        Serial.println("Error: Unknown command type. Use A, C, V, or P.");
      }

    } else {
      Serial.println("Invalid format!");
      Serial.println("Angles:    A,Leg,Coxa,Femur,Tibia  (e.g., A,0,90,90,90)");
      Serial.println(
          "Cartesian: C,Leg,X,Y,Z             (e.g., C,0,150,0,-50)");
      Serial.println("Velocity:  V,0,Angle,Speed,Rot     (e.g., V,0,90,100,0)");
      Serial.println("Posture:   P,0,Pitch,Roll,Height   (e.g., P,0,15,0,-20)");
    }
  }
}

void loop() {
  static int sequenceState = 0;
  static unsigned long lastActionTime = 0;

  // NEW: Custom X-distance for each leg.
  // Corner legs (0, 2, 3, 5) are pushed out to 130mm.
  // Middle legs (1, 4) stay at 100mm.
  float stanceRadius[6] = {80, 50.0, 80, 50, 80, 50};

  // STEP 1: Boot up and move to Idle
  if (sequenceState == 0) {
    for (int i = 0; i < 6; i++) {
      // Pull the idle pose slightly closer than the standing radius
      Vector3 idle = Vector3(stanceRadius[i] - 25.0, 0.0, 0.0);
      legs[i]->setTarget(idle, 1000);
    }
    lastActionTime = millis();
    sequenceState = 1;
  }

  // STEP 2: Stand up
  else if (sequenceState == 1 && (millis() - lastActionTime >= 1000)) {
    for (int i = 0; i < 6; i++) {
      // Stand at the designated custom radius
      Vector3 stand = Vector3(stanceRadius[i], 0.0, -80.0);
      legs[i]->setTarget(stand, 1000);
    }
    lastActionTime = millis();
    sequenceState = 2;
  }

  // STEP 3: Lock into Walking Mode
  else if (sequenceState == 2 && (millis() - lastActionTime >= 1000)) {
    sequenceState = 3;
  }

  // THE WALKING ENGINE

  if (sequenceState == 3) {
    unsigned long cycleTime = 2000;
    float globalPhase = (float)(millis() % cycleTime) / cycleTime;

    float phaseOffsets[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
    float legRotations[6] = {56.3, 0.0, -56.3, 56.3, 0.0, -56.3};
    float stanceRadius[6] = {80, 50, 80, 80, 50, 80};
    float strideOffsets[6] = {-40.0, 0.0, 40.0, -40.0, 0.0, 40.0};
    float sideMultiplier[6] = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0};

    // NEW: The tangent angle each foot must travel to rotate the body Clockwise
    // E.g., The Mid-Right leg (1) must walk backward (180 deg) to spin the body
    // CW.
    float tangentAngles[6] = {135.0, 180.0, -135.0, 45.0, 0.0, -45.0};

    Vector2 shoulderPivot = Vector2(-31.4, 0.0);

    for (int i = 0; i < 6; i++) {
      float legPhase = globalPhase + phaseOffsets[i];
      if (legPhase >= 1.0)
        legPhase -= 1.0;

      // 1. Calculate Global Translation Vector (X = Right, Y = Forward)
      float transX = sin(radians(globalWalkAngle)) * globalStrideLength;
      float transY = cos(radians(globalWalkAngle)) * globalStrideLength;

      // 2. Calculate Global Rotation Vector
      float rotX = sin(radians(tangentAngles[i])) * globalRotationSpeed;
      float rotY = cos(radians(tangentAngles[i])) * globalRotationSpeed;

      // 3. Sum the Vectors to get the Final Travel Path for this specific leg
      float finalX = transX + rotX;
      float finalY = transY + rotY;

      // 4. Convert back to a final Speed and Angle
      float finalSpeed = sqrt(finalX * finalX + finalY * finalY);

      // Using atan2(X, Y) perfectly maps our compass (0 = Forward, 90 = Right)
      float finalAngle = degrees(atan2(finalX, finalY));

      // 5. Generate the Trajectory
      float liftHeight = (finalSpeed > 5.0) ? 40.0 : 0.0;
      Vector3 point =
          generateTrajectory(legPhase, stanceRadius[i], 80.0, finalSpeed,
                             liftHeight, strideOffsets[i]);

      // 6. Apply Yaw and Leg Rotations
      float localWalkAngle = finalAngle * sideMultiplier[i];
      Vector2 footRestingCenter(stanceRadius[i], strideOffsets[i]);
      point = point.rotate(localWalkAngle, footRestingCenter);
      point = point.rotate(legRotations[i], shoulderPivot);

      // -------------------------------------------------------------------
      // 7. NEW: Apply Body Kinematics (Pitch, Roll, and Ride Height)
      // -------------------------------------------------------------------

      // Ride Height: Simply shift the Z target up or down
      point.z -= globalRideHeight;

      // Pitch: If the shoulder is far forward (Positive Y) and the body pitches
      // UP, the shoulder physically rises. We must push the foot target DOWN
      // (negative Z) to compensate.
      float pitchShift = bodyOffsetY[i] * sin(radians(globalPitch));
      point.z += pitchShift;

      // Roll: If the shoulder is far right (Positive X) and the body rolls
      // RIGHT, the shoulder physically drops. We must pull the foot target UP
      // (positive Z) to compensate.
      float rollShift = bodyOffsetX[i] * sin(radians(globalRoll));
      point.z -= rollShift;

      legs[i]->setInstantIK(point);
    }
  }

  handleSerialCommands();
}
