#include <Servo.h>
#include <AccelStepper.h>

// ---------------------------------------------------
// UTILS

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float constrainFloat(float x, float outMin, float outMax) {
  if (x > outMax) {
    return outMax;
  }
  if (x < outMin) {
    return outMin;
  }
  return x;
}
// ---------------------------------------------------
// CONSTANTS

// Yaw Swivel states
#define SWIVEL_IDLE 0                   // The swivel is idle.
#define SWIVEL_CALIBRATE_BOUNDS_LOW 1   // Swivel is calibrating a lower bound.
#define SWIVEL_CALIBRATE_BOUNDS_HIGH 2  // Swivel is calibrating an upper bound. 
#define SWIVEL_MOVING 3                 // Swivel moving towards a position.

// Yaw Swivel constants
#define SWIVEL_STEP_ENABLE 7
#define SWIVEL_STEP_PIN 9
#define SWIVEL_DIRECTION_PIN 8
#define SWIVEL_MICROSTEP_PIN_1 4
#define SWIVEL_MICROSTEP_PIN_2 5
#define SWIVEL_MICROSTEP_PIN_3 6
#define SWIVEL_LIMIT_LOW_PIN 2
#define SWIVEL_LIMIT_HIGH_PIN 4

// Launcher states
#define LAUNCHER_IDLE 0               // The launcher is idle.
#define LAUNCHER_EXTENDING 1          // Launcher is extending, to release a ball.
#define LAUNCHER_RETRACTING 2         // Launcher is retracting/resetting.

// Launcher Constants
#define LAUNCHER_EXTENDED_POS 10      // "Extended" servo position
#define LAUNCHER_RETRACTED_POS 155    // "Retracted" servo position
#define LAUNCHER_SERVO_PIN 10         // What pin the launcher servo is on
#define LAUNCHER_EXTEND_DURATION 500  // Time it takes the launcher servo to extend
#define LAUNCHER_RETRACT_DURATION 500 // Time it takes the launcher servo to retract

// Ball Detector constants
#define BALL_DETECTOR_PIN 15

// ---------------------------------------------------
// GLOBAL VARIABLES

// Launcher variables & state 
int iLauncherState = LAUNCHER_IDLE;
unsigned long lLauncherLastAction = 0;
bool bWantsLaunch = false;
bool bHasCommand = false;
Servo launcherServo; 

// Ball Detector state
bool bHasBall = false;
bool bHasBallLatch = false;

// Swivel variables & state
int iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;
long lLastMessageTimestamp = 0;
long lSwivelHighPos = 0;
long lSwivelNewTargetPos = 0;
long lSwivelTargetPos = 0;
float fSwivelLastYawMsg = 0;
long lLauncherExtendDuration = LAUNCHER_EXTEND_DURATION;
bool bSwivelClearedEndStop = false;
bool bYawIsReady = false;
bool bHasNewCommand = false;
bool bInitiallyConnecting = true;
AccelStepper stepper(AccelStepper::DRIVER, SWIVEL_STEP_PIN, SWIVEL_DIRECTION_PIN);

// Heartbeat (serial out)
unsigned long lLastHeartbeatMsg = 0;

// Serial in command buffer
static unsigned char cBytesRead = 0;
static unsigned char cSerialBuffer[64];

void(* resetFunc) (void) = 0;

// ---------------------------------------------------
// CALLBACK SETUP

void triggerCommandCallback() {
  bWantsLaunch = true;
}

void yawCommandCallback(float yaw_msg) {
  // Yaw message will be a value from -90 to 90 inclusive
  // Convert this to a target position.
  lSwivelNewTargetPos = (int)round(constrainFloat(mapFloat(-yaw_msg, -45.0f, 45.0f, 0.0f, (float)lSwivelHighPos),0.0f, (float)lSwivelHighPos));
  fSwivelLastYawMsg = yaw_msg;

  if (lSwivelNewTargetPos != lSwivelTargetPos || bHasCommand == false) {
    lSwivelTargetPos = lSwivelNewTargetPos;
    bYawIsReady = false;
    bHasCommand = true;
    bHasNewCommand = true;
  }
}

void resetCommandCallback() {
    // Completely reset the system to get out of a bad state.
    resetFunc();
}

// ---------------------------------------------------
// SETUP

void writeStatusMessage() {
  // Write a status message (comma separated):
  // swivel_state,launcher_state,stepper_pos,has_command,is_ready,has_ball

  bool isReady = bYawIsReady && bHasCommand;
  Serial.print(iSwivelState);
  Serial.print(",");
  Serial.print(iLauncherState);
  Serial.print(",");
  Serial.print(stepper.currentPosition());
  Serial.print(",");
  Serial.print(lSwivelTargetPos);
  Serial.print(",");
  Serial.print(bHasCommand);
  Serial.print(",");
  Serial.print(isReady);
  Serial.print(",");
  Serial.print(bHasBall);
  Serial.println();
}

void setup() {
  // Initialize launcher servo
  launcherServo.attach(LAUNCHER_SERVO_PIN);
  launcherServo.write(LAUNCHER_RETRACTED_POS);

  // Initialize stepper motor
  pinMode(SWIVEL_STEP_ENABLE, OUTPUT);
  digitalWrite(SWIVEL_STEP_ENABLE, HIGH);
  
  // Set microstepping
  pinMode(SWIVEL_MICROSTEP_PIN_1, OUTPUT);
  pinMode(SWIVEL_MICROSTEP_PIN_2, OUTPUT);
  pinMode(SWIVEL_MICROSTEP_PIN_3, OUTPUT);
  digitalWrite(SWIVEL_MICROSTEP_PIN_1, LOW);
  digitalWrite(SWIVEL_MICROSTEP_PIN_2, LOW);
  digitalWrite(SWIVEL_MICROSTEP_PIN_3, HIGH);
  
  // Stepper limit switches
  pinMode(SWIVEL_LIMIT_LOW_PIN, INPUT_PULLUP);
  pinMode(SWIVEL_LIMIT_HIGH_PIN, INPUT_PULLUP);

  // Ball detector
  pinMode(BALL_DETECTOR_PIN, INPUT_PULLUP);
  
  // Set max speed and acceleration for the stepper
  stepper.setSpeed(1500);
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(2000);

  // Start calibration mode
  iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;

  // Set launcher extend duration to default
  lLauncherExtendDuration = LAUNCHER_EXTEND_DURATION;

  // Start serial
  Serial.begin(115200);
}

// ---------------------------------------------------
// MAIN LOOP

void loop() {
  // Read serial in buffer for new commands
  // (TODO)
  while (!Serial) {
    // Do nothing as long as serial is not present.
    delay(10);
    return;
  }

  // Check limit switches
  bool limitLow = !digitalRead(SWIVEL_LIMIT_LOW_PIN);
  bool limitHigh = !digitalRead(SWIVEL_LIMIT_HIGH_PIN);

  // Check ball detector
  // Latch so once it's on, needs to be reset manually
  bHasBall = !digitalRead(BALL_DETECTOR_PIN) || bHasBallLatch;

  // Read in command buffer
  while (Serial.available()) {
    int inputChar = Serial.read();
    if (inputChar == '\n') {
      // Newline; parse command and execute, clear buffer
      cBytesRead += 1;
      cSerialBuffer[cBytesRead] = '\n';
      Serial.write(cSerialBuffer, cBytesRead + 1);
      cBytesRead = 0;
  
      // Split and parse command string
      String command = cSerialBuffer;
      command.trim();
      String commandId = getValue(command,',',0);
      String commandPayload = getValue(command,',',1);
      commandId.trim();
      commandPayload.trim();

      Serial.print("Command:");
      Serial.println(commandId);
      Serial.println(commandPayload);
      if (commandId == "z") {
        if (commandPayload.equals("stop")) {
          iSwivelState = SWIVEL_IDLE;
          Serial.println("Stopping calibration");
        }
        if (commandPayload.equals("start")) {
          iSwivelState = SWIVEL_CALIBRATE_BOUNDS_LOW;
        }
        if (commandPayload.equals("limit")) {
          if (iSwivelState == SWIVEL_CALIBRATE_BOUNDS_LOW) {
            limitLow = true;
          } else if (iSwivelState == SWIVEL_CALIBRATE_BOUNDS_HIGH) {
            limitHigh = true;
          }
        }
      }

      if (commandId == "r") {
        Serial.println("Resetting");
        bHasCommand = false;
        bYawIsReady = false;
        bWantsLaunch = false;
        bHasBall = false;
        bHasBallLatch = false;
      }
      
      if (commandId == "l") {
        Serial.println("Launching");
        triggerCommandCallback();
        if (commandPayload.equals("slow")) {
          lLauncherExtendDuration = 10000;
        }
      }

      if (commandId == "m") {
        Serial.print("Moving to: ");
        Serial.println(commandPayload.toFloat());
        yawCommandCallback(commandPayload.toFloat());
      }

      // Clear last command
      cBytesRead = 0;
      for (int i = 0; i < 64; i++) {
        cSerialBuffer[i] = 0;
      }
    }
    else if (cBytesRead < 64) {
      // Append command to buffer.
      cSerialBuffer[cBytesRead] = inputChar;
      cBytesRead++;
    } else {
      // Buffer overflowed, ignore packet until we receive a newline.
    }
  }

  // Write status message at 10Hz
  if (abs(millis() - lLastMessageTimestamp) > 100) {
    writeStatusMessage();
    lLastMessageTimestamp = millis();
  }
    
  if (bInitiallyConnecting) {
    bInitiallyConnecting = false;
    bYawIsReady = false;
  }

  switch (iSwivelState) {
    case SWIVEL_IDLE: {
      if (bHasNewCommand) {
        iSwivelState = SWIVEL_MOVING;
        bYawIsReady = false;
      } else {
        bYawIsReady = true;
      }

      // Send ball status while we are idle
      if (bHasBall) {
        bHasBallLatch = true;
      }
      
      break;
    }
    case SWIVEL_CALIBRATE_BOUNDS_LOW: {
      stepper.moveTo(-10000000);
      stepper.run();
      if (limitLow) {
        stepper.stop();
        iSwivelState = SWIVEL_CALIBRATE_BOUNDS_HIGH;
        stepper.setCurrentPosition(0);
        
        bYawIsReady = false;
      }
      break;
    }
    case SWIVEL_CALIBRATE_BOUNDS_HIGH: {
      stepper.moveTo(10000000);
      stepper.run();
      if (limitHigh) {
        stepper.stop();
        lSwivelHighPos = stepper.currentPosition();
        stepper.setCurrentPosition(lSwivelHighPos);
        stepper.run();
        
        // set target pos to middle of range
        lSwivelTargetPos = lSwivelHighPos / 2;
        bSwivelClearedEndStop = false;
        
        iSwivelState = SWIVEL_MOVING;
        bYawIsReady = false;
      }
      break;
    }
    case SWIVEL_MOVING: {
      stepper.moveTo(lSwivelTargetPos);
      stepper.run();
      if (!stepper.isRunning()) {
        iSwivelState = SWIVEL_IDLE;
        bHasNewCommand = false;
        bYawIsReady = true;
      }
      break;
    }
  }
  
  switch (iLauncherState) {
    case LAUNCHER_IDLE: {
      launcherServo.write(LAUNCHER_RETRACTED_POS);
      if (bWantsLaunch && iSwivelState == SWIVEL_IDLE) {
        launcherServo.write(LAUNCHER_EXTENDED_POS);
        iLauncherState = LAUNCHER_EXTENDING;
        lLauncherLastAction = millis();
      }
      break;
    }
    case LAUNCHER_EXTENDING: {
      // Wait while the launcher is extending, then start retraction.
      if (millis() - lLauncherLastAction > lLauncherExtendDuration) {
        launcherServo.write(LAUNCHER_RETRACTED_POS);
        iLauncherState = LAUNCHER_RETRACTING;
        lLauncherLastAction = millis();
      }
      break;
    }
    case LAUNCHER_RETRACTING: {
      // Wait while the launcher is retracting, then return to idle.
      if (millis() - lLauncherLastAction > LAUNCHER_RETRACT_DURATION) {
        iLauncherState = LAUNCHER_IDLE;
        bWantsLaunch = false;

        // Reset "Has ball" status after we have launched the ball
        bHasBall = false;
        bHasBallLatch = false;
      }
      break;
    }
  }
}
