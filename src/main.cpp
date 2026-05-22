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

Leg *legs[6];

Vector3 generateTrajectory(float phase, float defaultX, float standingHeight,
                           float stepLength, float stepHeight) {
  // Ensure phase stays strictly wrapped between 0.0 and 1.0
  phase = phase - floor(phase);

  Vector3 targetPos;

  // Y-axis: To walk forward, the leg swings forward in the air, and pushes
  // backward on the ground.
  float yLiftoff = -stepLength / 2.0;  // Back of the stride
  float yTouchdown = stepLength / 2.0; // Front of the stride
  float zGround = -standingHeight;

  // ---------------------------------------------------------
  // SWING PHASE: (0.0 to 0.5) - Foot in the air moving forward
  // ---------------------------------------------------------
  if (phase < 0.5) {
    float t = phase * 2.0;

    // 1. KINEMATIC SMOOTHING: Cosine Ease-in / Ease-out
    // Forces velocity to 0 at the corners to eliminate jerks
    float easedT = (1.0 - cos(t * PI)) / 2.0;

    // 2. GEOMETRIC SMOOTHING: Align P1 and P2 vertically
    Vector3 P0(defaultX, yLiftoff, zGround);
    Vector3 P3(defaultX, yTouchdown, zGround);

    float zControl = zGround + (stepHeight * 1.333);
    Vector3 P1(defaultX, yLiftoff, zControl); // Pulls straight up from liftoff
    Vector3 P2(defaultX, yTouchdown,
               zControl); // Drops straight down to touchdown

    // Calculate curve using the eased time variable
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

    // Apply the exact same smoothing to the ground phase
    float easedT = (1.0 - cos(t * PI)) / 2.0;

    // Move linearly from the front (touchdown) back to the rear (liftoff)
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
    input.trim(); // Remove stray whitespace

    char cmdType;
    int legIdx;
    float v1, v2, v3;

    // Parse format: Type, LegIndex, Value1, Value2, Value3
    if (sscanf(input.c_str(), "%c,%d,%f,%f,%f", &cmdType, &legIdx, &v1, &v2,
               &v3) == 5) {

      // Validate the leg index
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
        // Move to the target XYZ over 1000 milliseconds
        legs[legIdx]->setTarget(Vector3(v1, v2, v3), 1000);

        Serial.print("Leg ");
        Serial.print(legIdx);
        Serial.print(" TARGET set to -> X: ");
        Serial.print(v1);
        Serial.print(" | Y: ");
        Serial.print(v2);
        Serial.print(" | Z: ");
        Serial.println(v3);
      } else {
        Serial.println("Error: Unknown command type. Use 'A' or 'C'.");
      }

    } else {
      Serial.println("Invalid format!");
      Serial.println(
          "For Angles:    A,Leg,Coxa,Femur,Tibia  (e.g., A,0,90,90,90)");
      Serial.println(
          "For Cartesian: C,Leg,X,Y,Z             (e.g., C,0,150,0,-50)");
    }
  }
}

void loop() {
  static int sequenceState = 0;
  static unsigned long lastActionTime = 0;

  // STEP 1: Boot up and move to Idle (Tucked in)
  if (sequenceState == 0) {
    Vector3 idle = Vector3(75, 0.0, 0.0);
    for (int i = 0; i < 6; i++) {
      legs[i]->setTarget(idle, 1000);
    }
    lastActionTime = millis();
    sequenceState = 1;
  }

  // STEP 2: Stand up (Wait 1000ms after Step 1)
  else if (sequenceState == 1 && (millis() - lastActionTime >= 1000)) {
    Vector3 stand = Vector3(100, 0.0, -80);
    for (int i = 0; i < 6; i++) {
      legs[i]->setTarget(stand, 1000);
    }
    lastActionTime = millis();
    sequenceState = 2;
  }

  // STEP 3: Begin Walking (Wait 1000ms after Step 2)
  else if (sequenceState == 2 && (millis() - lastActionTime >= 1000)) {
    sequenceState = 3; // Lock into the continuous walking state
  }

  // THE WALKING ENGINE (Runs continuously once State 3 is reached)
  if (sequenceState == 3) {
    unsigned long cycleTime = 2000; // 2 seconds per full stride
    float globalPhase = (float)(millis() % cycleTime) / cycleTime;

    // The Tripod Phase Offsets
    float phaseOffsets[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};

    for (int i = 0; i < 6; i++) {
      // Add the leg's specific offset to the global clock
      float legPhase = globalPhase + phaseOffsets[i];

      // Wrap the phase so it always stays between 0.0 and 1.0
      if (legPhase >= 1.0) {
        legPhase -= 1.0;
      }

      // Generate the exact coordinate (X=100, Z_stand=80, Stride=100, Lift=40)
      Vector3 point = generateTrajectory(legPhase, 100.0, 80.0, 100.0, 40.0);

      // Push instantly to the hardware (the Leg class handles the 50Hz
      // limiting)
      legs[i]->setInstantIK(point);
    }
  } else {
    // Only run the point-to-point interpolator if we are in States 1 or 2
    for (int i = 0; i < 6; i++) {
      legs[i]->update();
    }
  }

  // Always listen for serial commands
  handleSerialCommands();
}
