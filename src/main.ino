#include math.h

const int echoPin = 2;
const int triggerPin = 3;

const int maxRange =  4000; // limit of sensor; in mm
const int deskMinDistance = 100; //lowest desk position based on limits; in mm
const int deskMaxDistance = 1000; // highest desk position basted on limits; in mm
const int deskLowPosition = 300; // target sitting height; in mm
const int deskHighPosition = 750; // target standing height; in mm

void setup(){
  pinMode(echoPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  serial.begin(9600);
}

void loop{
  int currentDistance = 0;
  int newDistance = 0;
  int motionDelta = 15;
  int direction = 0;
  boolean deskInMotion = false;
  currentDistance = sonicRangeFinder();
  delay(10);
  newDistance = sonicRangeFinder();
  if (abs(currentDistance - newDistance) > motionDelta) {
    deskInMotion = true;
  }
  while (deskInMotion) {

  }
}

void moveDesk(int direction) {
  
}

int sonicRangeFinder() {
  // fires the ultrasonic range finder and then returns the distance in mm.
  unsigned long pulseWidth = 0;
  unsigned long pulseT1 = 0;
  unsigned long pulseT2 = 0;
  int distance = 0; // in mm
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
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
