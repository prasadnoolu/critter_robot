/* Use this with Version 1.0 of Critter App
  Created by Slant Concepts
  Published 10/24/2017
*/

#define echoPin A2 // Echo Pin
#define trigPin A3 // Trigger Pin

#include <Servo.h>  //arduino library
#include <math.h>   //standard c library

Servo rightKnee;
Servo rightShoulder;
Servo leftKnee;
Servo leftShoulder;

// Structure for the legs. Currently used only within functions
struct legPos {
  int shoulder;
  int knee;
};

// Pure variable hodlers of leg values
int rightShoulderAngle = 0;
int rightKneeAngle = 0;
int leftShoulderAngle = 0;
int leftKneeAngle = 0;

int robotCommand = 190; //default command is standing

struct legPos rightLeg;
struct legPos leftLeg;
int desiredDelay = 3;

bool holder = 1;

int maximumRange = 200;   // Maximum range needed
int minimumRange = 0;     // Minimum range needed

long readDistance;        // the output distance from the sensor
int nothingCount;         //the number of times the sensor never sees anything
long randomNumber = 0;    // random number fomr 1-10

//++++++++++++++++++++++++Function Declarations+++++++++++++++

int ultraSensor(int theEchoPin, int theTrigPin);
void moveTo( struct legPos rightLeg, struct legPos leftLeg, int desiredDelay);
int servoParallelControl (int thePos, Servo theServo, int theSpeed );

//motion functions
int whatAction( int whichMotion, int servoSpeed);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  rightKnee.attach(3);        // attaches the servo on pin 9 to the servo object
  rightShoulder.attach(2);
  leftShoulder.attach(4);
  leftKnee.attach(5);

  rightShoulder.write(157);
  rightKnee.write(166);
  leftShoulder.write(30);
  leftKnee.write(6);

  Serial.println("started");
}

void loop() {

  if (Serial.available()) {
    //ready = 1;
    rightShoulderAngle = Serial.parseInt();
    rightKneeAngle = Serial.parseInt();
    leftShoulderAngle = Serial.parseInt();
    leftKneeAngle = Serial.parseInt();
    desiredDelay = Serial.parseInt();

    if (Serial.read() == '\n') {            // if the last byte is 'd' then stop reading and execute command 'd' stands for 'done'
      Serial.flush();                     //clear all other commands piled in the buffer
      Serial.print('d');                  //send completion of the command
    }

  } //end of serial read

  //First Check if the rightShoulder value is above 180. If so then interpret it as a robot mode command
  if (rightShoulderAngle > 180) {
    robotCommand = rightShoulderAngle;


    // perform the action or enter the mode defined.
    if (robotCommand == 190) {
      //Wait for next command
      // this is a holding loop
    }

    if (robotCommand == 191) {
      // Step
      whatAction(1, desiredDelay);
    }

    if (robotCommand == 192) {
      // Backup
      whatAction(2, desiredDelay);
    }

    if (robotCommand == 193) {
      // Right
      whatAction(3, desiredDelay);
    }

    if (robotCommand == 194) {
      // Left
      whatAction(4, desiredDelay);
    }

    if (robotCommand == 195) {
      // Dance
      whatAction(5, desiredDelay);
    }

    if (robotCommand == 196) {
      // Wave
      whatAction(6, desiredDelay);
    }

    if (robotCommand == 197) {
      // Clap
      whatAction(7, desiredDelay);
    }



    //+++++++++++++++++++++ Wander Mode - generates random behavior while avoiding abstacles
    if (robotCommand == 300) {

      //read the distance read by the sensor
      readDistance = ultraSensor(echoPin, trigPin);
      //Serial.println(readDistance);

      if (readDistance > 20) {
        //don't worry about it
        //maybe count the number of time and
        //Serial.println("just walking dont see anything");
        whatAction(1, 3); // continue to walk
        nothingCount ++;
        if (nothingCount >= 20) {
          //do something

          nothingCount = 0;
        }
      }

      else if (readDistance <= 6) {
        //stand and backup
        //enter a functio with the trained arrayes stored
        // sequence through those array by calling the
        //similar function to if dist =10
        //Serial.println("in close");
        whatAction(2, 3);
      }

      else if (readDistance <= 20 && readDistance > 6) {
        //this is where all the fun starts
        randomNumber = random(3);
        //Serial.println(randomNumber);
        if (randomNumber == 1) {
          //Serial.println("turnright");
          whatAction(3, 3);
        }

        if (randomNumber == 2) {
          //Serial.println("turnleft");
          whatAction(4, 3);
        }

      }

      else {
        whatAction(1, 3);
      }

    } // end of Wander Mode
  } // end of >180 options

  // ++++++++++++++++++++++++ When app is just sending joint angle commands
  else {
    struct legPos rightLeg;
    struct legPos leftLeg;
    int aServoSpeed;

    rightLeg.shoulder = rightShoulderAngle;
    rightLeg.knee = rightKneeAngle;
    leftLeg.shoulder = leftShoulderAngle;
    leftLeg.knee = leftKneeAngle;
    aServoSpeed  = desiredDelay;

    moveTo (rightLeg, leftLeg, aServoSpeed);
  }
}

//++++++++++++++++++++++FUNCTION DEFINTIIONS+++++++++++++++++++++

int ultraSensor(int theEchoPin, int theTrigPin) {
  //this fucntion caluclates and returns the distance in cm

  long duration, distance; // Duration used to calculate distance
  /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(theTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(theTrigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(theTrigPin, LOW);
  duration = pulseIn(theEchoPin, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration / 58.2;
  return distance;

}

//+++++++++++++++
int whatAction( int whichMotion, int servoSpeed) {

  int waypoints = 0; // iterate through all waypoints int he trajectory
  int aJoint = 0;  //iterate from shoulde rto speed
  int aSpeed;

  struct legPos rightLeg;
  struct legPos leftLeg;

  switch (whichMotion) {
    case 1: {
        //walk
        int trajectory[5][5] = {{ 155, 43, 35, 135, 6}, {155, 175, 35, 5, 6}, {92, 175, 76, 5, 6}, {92, 40, 76, 120, 6}, {130, 40, 47, 120, 6}};

        int trajSize =  5; 
        // must place code in the case with the trajectory else it is not recognized by the compiler
        // this is poor code but it the best for this situation
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;

          // Serial.print(rightLeg.shoulder);
          //  Serial.print(',');
          // Serial.print(rightLeg.knee);
          // Serial.print(',');
          // Serial.print(leftLeg.shoulder);
          // Serial.print(',');
          // Serial.print(leftLeg.knee);
          // Serial.print(',');
          // Serial.println(aSpeed);
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;
        }

        break;
      }

    case 2: {
        // backward
        int trajectory[5][5] = {{ 138, 5, 30, 175, 6}, {61, 5, 118, 175, 6}, {61, 170, 118, 5, 6}, {158, 170, 13, 5, 6}, (158, 5, 13, 168, 6)};

        int trajSize =  4;
        // must place code in the case with the trajectory else it is not recognized by the compiler
        // this is porr code but it the best for this situation
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;

          // Serial.print(rightLeg.shoulder);
          // Serial.print(',');
          // Serial.print(rightLeg.knee);
          // Serial.print(',');
          //  Serial.print(leftLeg.shoulder);
          //  Serial.print(',');
          //  Serial.print(leftLeg.knee);
          // Serial.print(',');
          // Serial.println(aSpeed);
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;

        }

        break;
      }

    case 3: {
        //right
        int trajectory[4][5] = {{ 74, 80, 17, 120, 3}, {74, 124, 17, 60, 3}, {159, 124, 122, 60, 3}, {159, 56, 122, 114, 3}};
        int trajSize =  4;
        // must place code in the case with the trajectory else it is not recognized by the compiler
        // this is porr code but it the best for this situation
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;

          // Serial.print(rightLeg.shoulder);
          // Serial.print(',');
          //Serial.print(rightLeg.knee);
          //Serial.print(',');
          // Serial.print(leftLeg.shoulder);
          //  Serial.print(',');
          //  Serial.print(leftLeg.knee);
          // Serial.print(',');
          // Serial.println(aSpeed);
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;

        }

        break;
      }

    case 4: {
        //left
        int trajectory[4][5] = {{ 122, 56, 159, 114, 3}, {122, 130, 159, 44, 3}, {18, 130, 63, 44, 3}, {18, 70, 63, 110, 3}};
        int trajSize =  4;
        // must place code in the case with the trajectory else it is not recognized by the compiler
        // this is poor code but it the best for this situation
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;

          //Serial.print(rightLeg.shoulder);
          //Serial.print(',');
          //Serial.print(rightLeg.knee);
          //Serial.print(',');
          //Serial.print(leftLeg.shoulder);
          // Serial.print(',');
          //Serial.print(leftLeg.knee);
          // Serial.print(',');
          // Serial.println(aSpeed);
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;
        }

        break;
      }

    case 5: {
        //Dance
        int trajectory[10][5] = {{ 90, 17, 91, 168, 6}, {90, 175, 91, 168, 6}, {90, 5, 91, 5, 6}, {90, 175, 91, 175, 6}, {90, 5, 91, 5, 6}, {90, 175, 91, 175, 6}, {90, 15, 91, 175, 6}, {90, 175, 91, 5, 6}, {90, 69, 91, 103, 6}, (90, 175, 91, 5, 6)};
        int trajSize =  10;
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;

        }
        break;
      }

    case 6: {
        //Wave
        int trajectory[8][5] = {{ 157, 25, 30, 84, 6}, {145, 25, 30, 84, 6}, {175, 25, 30, 84, 6}, {133, 25, 30, 84, 6}, {175, 25, 30, 84, 6}, {135, 25, 30, 84, 6}, {175, 25, 30, 84, 6}, {138, 25, 30, 84, 6}};
        int trajSize =  8;
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;

        }
        break;
      }

    case 7: {
        //Clap
        int trajectory[5][5] = {{ 175, 62, 5, 109, 6}, {128, 62, 48, 109, 6}, {175, 62, 5, 109, 6}, {138, 62, 56, 109, 6}, {175, 62, 5, 109, 6}};
        int trajSize =  5;
        while (waypoints < trajSize) {

          rightLeg.shoulder = trajectory[waypoints][aJoint];
          rightLeg.knee = trajectory[waypoints][aJoint + 1];
          leftLeg.shoulder = trajectory[waypoints][aJoint + 2];
          leftLeg.knee = trajectory[waypoints][aJoint + 3];
          aSpeed  = servoSpeed;
          moveTo (rightLeg, leftLeg, aSpeed);

          waypoints++;

        }
        break;
      }

    case 8: {
        //Wave
        break;
      }
  } // end switch

} // end the function


//+++++++++++++++++++++++++++++++++++++
void moveTo( struct legPos rightLeg, struct legPos leftLeg, int desiredDelay) {
  int status1 = 0;
  int status2 = 0;
  int status3 = 0;
  int status4 = 0;
  int done = 0 ;

  //Serial.println("in moveto");
  //Serial.print(rightLeg.shoulder);
  //Serial.print(',');
  //Serial.print(rightLeg.knee);
  //Serial.print(',');
  //Serial.print(leftLeg.shoulder);
  //Serial.print(',');
  //Serial.print(leftLeg.knee);
  //Serial.print(',');
  //Serial.println(desiredDelay);


  while ( done == 0) {
    //move the servo to the desired position
    //this loop will cycle through the servos sending each the desired position.
    //Each call will cause the servo to iterate about 1-5 degrees
    //the rapid cycle of the loop makes the servos appear to move simultaneously
    status1 = servoParallelControl(rightLeg.shoulder, rightShoulder, desiredDelay);
    status2 = servoParallelControl(rightLeg.knee,  rightKnee, desiredDelay);
    status3 = servoParallelControl(leftLeg.shoulder, leftShoulder, desiredDelay);
    status4 = servoParallelControl(leftLeg.knee, leftKnee, desiredDelay);

    //continue until all have reached the desired position
    if (status1 == 1 & status2 == 1 & status3 == 1 & status4 == 1) {
      done = 1;
    }

  }// end of while

}

//+++++++++++++++++++++++++++++++++++++++++

int servoParallelControl (int thePos, Servo theServo, int theSpeed ) {

  int startPos = theServo.read();        //read the current pos
  int newPos = startPos;

  //define where the pos is with respect to the command
  // if the current position is less that the actual move up
  if (startPos < (thePos - 5)) {

    newPos = newPos + 1;
    theServo.write(newPos);
    delay(theSpeed);
    return 0;

  }

  else if (newPos > (thePos + 5)) {

    newPos = newPos - 1;
    theServo.write(newPos);
    delay(theSpeed);
    return 0;

  }

  else {
    return 1;
  }

}



