#include "Adafruit_PWMServoDriver.h"
#include "Legs.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Bluepad32.h>
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

// Positive Y is forward, Negative Y is backward. Positive X is right, Negative
// X is left.
float bodyOffsetY[6] = {121.0, 0.0, -121.0, 121.0, 0.0, -121.0};
float bodyOffsetX[6] = {80.7, 97.3, 80.7, -80.7, -97.3, -80.7};
Leg *legs[6];

// --- Bluepad32 Controller Pointer ---
ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("XBOX CONTROLLER CONNECTED!");
    myController = ctl;

    myController->playDualRumble(0, 250, 0x80, 0x40);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("XBOX CONTROLLER DISCONNECTED!");
    myController = nullptr;

    // Safety lock: Stop the robot immediately if the controller dies
    globalStrideLength = 0.0;
    globalRotationSpeed = 0.0;
  }
}

// --- Dynamic Gait Engine ---
int currentGait = 0; // 0 = Tripod, 1 = Ripple, 2 = Wave
float globalDutyFactor = 0.5;
float globalPhaseOffsets[6] = {0.0, 0.5, 0.0,
                               0.5, 0.0, 0.5}; // Defaults to Tripod
String gaitNames[3] = {"TRIPOD", "RIPPLE", "WAVE"};

// Helper function to swap gaits
void setGait(int gaitIndex) {
  currentGait = gaitIndex;
  Serial.print("GAIT SWITCHED TO: ");
  Serial.println(gaitNames[currentGait]);

  if (currentGait == 0) {
    globalDutyFactor = 0.5; // TRIPOD: 2 Groups of 3
    float tripod[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
    memcpy(globalPhaseOffsets, tripod, sizeof(tripod));
  } else if (currentGait == 1) {
    globalDutyFactor = 0.666;
    float ripple[6] = {0.0, 0.333, 0.666, 0.333, 0.666, 0.0};
    memcpy(globalPhaseOffsets, ripple, sizeof(ripple));
  } else if (currentGait == 2) {
    globalDutyFactor = 0.833;
    float wave[6] = {0.333, 0.166, 0.0, 0.833, 0.666, 0.5};
    memcpy(globalPhaseOffsets, wave, sizeof(wave));
  }
}

Vector3 generateTrajectory(float phase, float defaultX, float standingHeight,
                           float stepLength, float stepHeight, float yOffset,
                           float dutyFactor) {
  // Ensure phase stays strictly wrapped between 0.0 and 1.0
  phase = phase - floor(phase);

  // Map the boundaries of the stride
  float yLiftoff = (-stepLength / 2.0) + yOffset;  // Back of the stride
  float yTouchdown = (stepLength / 2.0) + yOffset; // Front of the stride
  float zGround = -standingHeight;                 // Negative Z is the floor

  float y = 0.0;
  float z = zGround;

  // ---------------------------------------------------------
  // SWING PHASE: (0.0 to 0.5) - Foot in the air moving forward
  // ---------------------------------------------------------
  float swingPhase = 1.0 - dutyFactor;
  if (phase < swingPhase) {
    // Normalize time from 0.0 to 1.0 for the swing curve
    float t = phase / swingPhase;

    // Standard linear time for a true Bezier velocity profile
    float u = 1.0 - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    // Y-Axis Bezier Points (Back to Front)
    float p0y = yLiftoff;
    float p1y =
        yLiftoff + (stepLength * 0.1); // Pull forward slightly on liftoff
    float p2y = yTouchdown - (stepLength * 0.1); // Ease gently into touchdown
    float p3y = yTouchdown;

    // Z-Axis Bezier Points (Ground -> Up -> Ground)
    float p0z = zGround;
    float p1z =
        zGround +
        (stepHeight * 1.333); // The 1.3 multiplier creates the teardrop snap
    float p2z = zGround + (stepHeight * 1.333);
    float p3z = zGround;

    // Calculate absolute position on the curve
    y = uuu * p0y + 3 * uu * t * p1y + 3 * u * tt * p2y + ttt * p3y;
    z = uuu * p0z + 3 * uu * t * p1z + 3 * u * tt * p2z + ttt * p3z;
  }
  // ---------------------------------------------------------
  // STANCE PHASE: (0.5 to 1.0) - Foot on the ground pushing back
  // ---------------------------------------------------------
  else {
    // Normalize time from 0.0 to 1.0 for the ground pull
    float t = (phase - swingPhase) / dutyFactor;
    // Linear mapping for a perfectly constant chassis speed
    y = yTouchdown - (stepLength * t);
    z = zGround;
  }

  return Vector3(defaultX, y, z);
}

void handleGamepad() {
  BP32.update();

  if (myController && myController->isConnected()) {

    // 1. Left Stick (Translation - Walk/Strafe)
    float stickX = myController->axisX() / 512.0;
    float stickY = -(myController->axisY() / 512.0);

    if (abs(stickX) < 0.20)
      stickX = 0;
    if (abs(stickY) < 0.20)
      stickY = 0;

    float magnitude = sqrt((stickX * stickX) + (stickY * stickY));
    globalStrideLength = min(magnitude * 80.0, 80.0);

    if (globalStrideLength > 0) {
      globalWalkAngle = degrees(atan2(stickX, stickY));
    }

    // ---------------------------------------------------------
    // 2. Right Stick & Bumper Logic (Spin vs. Tilt)
    // ---------------------------------------------------------
    if (myController->r1()) {
      // SHIFT MODE: Right Bumper is HELD. Right Stick controls Posture.
      globalRotationSpeed = 0.0; // Prevent spinning while tilting

      float rightStickX = myController->axisRX() / 512.0;
      float rightStickY = -(myController->axisRY() / 512.0); // Invert Y-axis

      // Increment Pitch and Roll based on stick position
      if (abs(rightStickX) > 0.2)
        globalRoll = rightStickX * 30;
      if (abs(rightStickY) > 0.2)
        globalPitch = rightStickY * 30;

    } else {
      // NORMAL MODE: Right Bumper is RELEASED. Right Stick controls Spin.
      float rotX = myController->axisRX() / 512.0;
      if (abs(rotX) < 0.15)
        rotX = 0;
      globalRotationSpeed = rotX * 100.0;
    }

    // 3. Right Stick Click (R3) - Instant Posture Reset
    if (myController->thumbR()) {
      globalPitch = 0.0;
      globalRoll = 0.0;
      // globalRideHeight = 0.0; // Uncomment this if you want R3 to reset
      // height too
    }

    // 4. D-PAD (Ride Height)
    if (myController->dpad() == DPAD_UP) {
      globalRideHeight += 1.0;
    } else if (myController->dpad() == DPAD_DOWN) {
      globalRideHeight -= 1.0;
    }
    globalRideHeight = constrain(globalRideHeight, -40.0, 40.0);

    static bool dpadLeftWasPressed = false;
    bool dpadLeftIsPressed = (myController->dpad() & DPAD_LEFT);

    // Edge detection: Only trigger the exact moment the button is pressed down
    if (dpadLeftIsPressed && !dpadLeftWasPressed) {
      int nextGait = (currentGait + 1) % 3; // Loops 0 -> 1 -> 2 -> 0
      setGait(nextGait);

      // Optional: Haptic feedback to confirm the shift!
      myController->playDualRumble(0, 150, 0x40, 0x00);
    }
    dpadLeftWasPressed = dpadLeftIsPressed;
  }
}

void setup() {
  Serial.begin(115200);
  sb1.begin();
  sb2.begin();
  sb1.setPWMFreq(60);
  sb2.setPWMFreq(60);
  delay(2000);
  Serial.println("Initializing Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  // Right Side (Board 1)
  legs[0] = new Leg(&sb1, 0, 1, 2, 95.0 - 40, 110.0 + 23, 22.0 - 20 + 11.3 - 90,
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
      Serial.println("Angles:    A,Leg,Coxa,Femur,Tibia  (e.g.,A,0,90,90,90)");
      Serial.println(
          "Cartesian: C,Leg,X,Y,Z             (e.g., C,0,150,0,-50)");
      Serial.println("Velocity:  V,0,Angle,Speed,Rot     (e.g.,V,0,90,100,0)");
      Serial.println("Posture:   P,0,Pitch,Roll,Height   (e.g., P,0,15,0,-20)");
    }
  }
}

void loop() {
  static int sequenceState = 0;
  static unsigned long lastActionTime = 0;
  unsigned long cycleTime = 2000;
  float globalPhase = (float)(millis() % cycleTime) / cycleTime;
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

  // ---------------------------------------------------------
  // STATE 3: THE WALKING ENGINE
  // ---------------------------------------------------------
  if (sequenceState == 3) {
    // Calculate the master time phase (Now driven by globalCycleTime from
    // the
    // trigger)
    float globalPhase = (float)(millis() % cycleTime) / cycleTime;

    float legRotations[6] = {56.3, 0.0, -56.3, 56.3, 0.0, -56.3};
    float stanceRadius[6] = {80.0, 50.0, 80.0, 80.0, 50.0, 80.0};
    float strideOffsets[6] = {-40.0, 0.0, 40.0, -40.0, 0.0, 40.0};
    float sideMultiplier[6] = {1.0, 1.0, 1.0, -1.0, -1.0, -1.0};
    float tangentAngles[6] = {135.0, 180.0, -135.0, 45.0, 0.0, -45.0};

    Vector2 shoulderPivot = Vector2(-31.4, 0.0);

    // Convert Pitch and Roll to radians once per frame to save CPU cycles
    float pitchRad = radians(globalPitch);
    float rollRad = radians(globalRoll);

    for (int i = 0; i < 6; i++) {
      // 1. Apply Dynamic Gait offsets (Tripod, Ripple, Wave)
      float legPhase = globalPhase + globalPhaseOffsets[i];
      if (legPhase >= 1.0)
        legPhase -= 1.0;

      // 2. Calculate Global Translation Vector (X = Right, Y = Forward)
      float transX = sin(radians(globalWalkAngle)) * globalStrideLength;
      float transY = cos(radians(globalWalkAngle)) * globalStrideLength;

      // 3. Calculate Global Rotation Vector (Yaw)
      float rotX = sin(radians(tangentAngles[i])) * globalRotationSpeed;
      float rotY = cos(radians(tangentAngles[i])) * globalRotationSpeed;

      // 4. Sum the Vectors to get the Final Travel Path
      float finalX = transX + rotX;
      float finalY = transY + rotY;

      float finalSpeed = sqrt((finalX * finalX) + (finalY * finalY));
      float finalAngle = degrees(atan2(finalX, finalY));

      // 5. Generate the Bezier Trajectory
      // Base Z height is 80.0. Only lift the foot if the robot is actually
      // commanded to move.
      float liftHeight = (finalSpeed > 5.0) ? 40.0 : 0.0;
      Vector3 point =
          generateTrajectory(legPhase, stanceRadius[i], 80.0, finalSpeed,
                             liftHeight, strideOffsets[i], globalDutyFactor);

      // 6. Apply Yaw and Leg Rotations
      float localWalkAngle = finalAngle * sideMultiplier[i];
      Vector2 footRestingCenter(stanceRadius[i], strideOffsets[i]);
      point = point.rotate(localWalkAngle, footRestingCenter);
      point = point.rotate(legRotations[i], shoulderPivot);

      // -------------------------------------------------------------------
      // 7. True 6-DoF Body Kinematics (Pitch, Roll, and Ride Height)
      // -------------------------------------------------------------------

      // A. Calculate vertical shoulder movement
      // Pitch > 0 (Nose Up): Front shoulders rise.
      // Roll  > 0 (Right Down): Right shoulders drop.
      float shoulder_dZ =
          (bodyOffsetY[i] * sin(pitchRad)) - (bodyOffsetX[i] * sin(rollRad));

      // B. Calculate lateral chassis sway
      // When the top plate tilts, the robot sways. We shift the feet to keep
      // the Center of Gravity locked. (point.z is our standing height
      // distance)
      float shoulder_dX = point.z * sin(rollRad);
      float shoulder_dY = point.z * sin(pitchRad);

      // C. Apply the Inverse Kinematic compensation
      point.x += shoulder_dX;
      point.y += shoulder_dY;

      // Subtracting the Z shift forces the foot down when the shoulder goes up
      point.z -= shoulder_dZ;

      // D. Apply Global Ride Height (D-Pad adjustment)
      point.z -= globalRideHeight;

      // 8. Push to the Inverse Kinematics Solver
      legs[i]->setInstantIK(point);
    }
  }
  handleGamepad();
}
