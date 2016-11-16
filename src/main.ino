/********************************************************************************
Desk controler sketch.



References and insiration:
sparkFun HC-SR04 sketch https://codebender.cc/sketch:356078#HC-SR04%20Ultrasonic%20Sensor%20Example.ino
IKEA moving desk http://www.ikea.com/us/en/catalog/products/S19022530/#
*******************************************************************************/

#include "Arduino.h"
#include "SPI.h"

const int echoPin = 2;
const int sensorTriggerPin = 3;
const int upSwitchPin = 4;
const int downSwitchPin = 5;
const int upTriggerPin = 6;
const int downTriggerPin = 7;
const int modePin = 8;


const int maxRange =  4000; // limit of sensor; in mm
const int motionDelta = 15; // Distance travel deadband
const int crashLimit = 10; // velocity threashold for crash detection. in mm/s

const int deskMinDistance = 560; //lowest desk position based on limits; in mm
const int deskMaxDistance = 1220; // highest desk position basted on limits; in mm
const int deskLowDistance = 750; // target sitting height; in mm
const int deskHighDistance = 1100; // target standing height; in mm
const int numberOfPositions = 4;
/*
position # | description
 0  | not at a preset position
 1  | Mechanical lower limit
 2  | Sitting position
 3  | Standing position
 4  | Mechanical upper limit
*/

void setup() {
  pinMode(echoPin, INPUT);
  pinMode(sensorTriggerPin, OUTPUT);
  pinMode(upSwitchPin, INPUT);
  pinMode(downSwitchPin, INPUT);
  pinMode(upTriggerPin, OUTPUT);
  pinMode(downTriggerPin, OUTPUT);
  pinMode(modePin, INPUT);


  digitalWrite(sensorTriggerPin, LOW);
  Serial.begin(9600);
}

void loop() {
  boolean upPress = false;
  boolean downPress = false;
  boolean modePreSet = true;
  int direction = 0;
  int distance = sonicRangeFinder();
  int position = deskPosition();
  if(position == 0) {
    if (distance > deskLowDistance + motionDelta) {
      position = 3;
    }
    else {
      position = 2;
    }
  }
  upPress = upButtonSense();
  downPress = downButtonSense();
  while (upPress ^ downPress) {
    if(modePreSet) {
      if(upPress) {
        direction = 1;
      }
      else {
        direction = -1;
      }
      moveDesk(direction, position);
    }
    else {
      if(upPress) {
        digitalWrite(upTriggerPin,HIGH);
      }
      else {
        digitalWrite(downTriggerPin,HIGH);
      }
    }
    upPress = upButtonSense();
    downPress = downButtonSense();
  }
  digitalWrite(upTriggerPin,LOW);
  digitalWrite(downTriggerPin,LOW);
}

boolean upButtonSense() {
  boolean buttonPress = false;
  if(digitalRead(upSwitchPin)) {
    buttonPress = true;
  }
  return buttonPress;
}

boolean downButtonSense() {
  boolean buttonPress = false;
  if(digitalRead(downSwitchPin)) {
    buttonPress = true;
  }
  return buttonPress;
}

int sonicRangeFinder() {
  // fires the ultrasonic range finder and then returns the distance in mm.
  unsigned long pulseWidth = 0;
  unsigned long pulseT1 = 0;
  unsigned long pulseT2 = 0;
  int distance = 0; // in mm
  digitalWrite(sensorTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTriggerPin, LOW);
  while (digitalRead(echoPin) == 0) {
    pulseT1 = micros();
    while (digitalRead(echoPin) == 1) {
      pulseT2 = micros();
      pulseWidth=pulseT2-pulseT1;
    }
  }
  distance = pulseWidth / 5.80;
  return distance;
}

boolean deskinMotion() {
  // returns whether the desk is currently moving or not
  int currentDistance = 0;
  int newDistance = 0;
  boolean deskMoving = false;
  currentDistance = sonicRangeFinder();
  delay(10);
  newDistance = sonicRangeFinder();
  if (abs(currentDistance - newDistance) > motionDelta) {
    deskMoving = true;
  }
  return deskMoving;
}

int deskPosition() {
  // returns the position number of where the desk is. will return a 0 if not in a position.
  int currentDistance = 0;
  int newDistance = 0;
  int averageDistance = 0;
  int deskPositionValue = 0;
  int currentSpeed = deskSpeed();
  currentDistance = sonicRangeFinder();
  delay(10);
  newDistance = sonicRangeFinder();
  averageDistance = (currentDistance + newDistance)/2;

  if (averageDistance <= deskLowDistance + motionDelta) {
    if (averageDistance <= deskMinDistance + motionDelta) {
      deskPositionValue = 1;
    }
    else if (averageDistance >= deskLowDistance - motionDelta) {
      deskPositionValue = 2;
    }
  }
  else if (averageDistance >= deskHighDistance - motionDelta) {
    if (averageDistance <= deskHighDistance + motionDelta) {
      deskPositionValue = 3;
    }
    else if (averageDistance >= deskMaxDistance - motionDelta) {
      deskPositionValue = 4;
    }
  }
  return deskPositionValue;
}

int deskSpeed() {
  int currentDistance = 0;
  int newDistance = 0;
  int deskVelocity = 0;
  int deltaTime = 0;
  unsigned long currentTime;
  unsigned long newTime;
  currentDistance = sonicRangeFinder();
  currentTime = millis();
  delay(10);
  newDistance = sonicRangeFinder();
  newTime = millis();
  deltaTime = newTime-currentTime;
  deskVelocity = (newDistance - currentDistance) / deltaTime / 1000;
  return deskVelocity;
}

void moveDesk(int direction, int originPosition) {
  //moves the desk up or down one position
  int currentDistance = 0;
  int currentSpeed = 0;
  int currentPosition = originPosition;
  int targetPosition = currentPosition + direction;
  boolean buttonPressed = false;
  if (targetPosition < 1 || targetPosition > numberOfPositions) {
    //handling out of bounds conditions
    if (targetPosition < 1) {
      targetPosition = 1;
    }
    else {
      targetPosition = numberOfPositions;
    }
  }
  currentDistance = sonicRangeFinder();
  currentSpeed = deskSpeed();
  while (currentPosition != targetPosition) {
      if(abs(currentSpeed) < crashLimit) {
        if (direction > 0) {
          digitalWrite(upTriggerPin, HIGH);
        }
        else {
          digitalWrite(downTriggerPin, HIGH);
        }
      }
      delay(250);
      currentSpeed = deskSpeed();
      currentPosition = deskPosition();
      //check if buttons are being pressed to abort move
      if(direction > 0) {
        buttonPressed = downButtonSense();
      }
      else {
        buttonPressed = upButtonSense();
      }
      if (buttonPressed) {
        //if you push the opposite direction button then the desk is moving, crash the desk.
        currentSpeed = 0;
      }
      if(currentPosition != targetPosition && abs(currentSpeed) < crashLimit) {
        //abort, then reverse move since desk has crashed.
        digitalWrite(upTriggerPin, LOW);
        digitalWrite(downTriggerPin, LOW);
        delay(500);
        targetPosition = originPosition;
        direction = direction * (-1);
      }
  }
  digitalWrite(upTriggerPin, LOW);
  digitalWrite(downTriggerPin, LOW);
}
