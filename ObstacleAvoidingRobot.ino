//The DIY Life
//Michael Klements
//29 June 2020
//Obstacle Avoiding Robot

#include <AFMotor.h>                              //Import library to control motor shield
#include <Servo.h>                                //Import library to control the servo

AF_DCMotor rightBack(2);                          //Create an object to control each motor
AF_DCMotor rightFront(3);
AF_DCMotor leftFront(4);
AF_DCMotor leftBack(1);
Servo servoLook;                                  //Create an object to control the servo

#define SERVO 10
#define TRIG 14                                    //Assign the ultrasonic sensor pins
#define ECHO 15
#define IR_LEFT 16                                  // Left infrared sensor
#define IR_RIGHT 17                                 // Right infrared sensor
#define MAX_DIST 150                                //Maximum sensing distance (Objects further than this distance are ignored)
#define STOP_DIST 50                                //Minimum distance from an object to stop in cm
#define MOTOR_SPEED 50                              //The maximum motor speed
#define TURN_SPEED 40                               //Amount to add to motor speed when turning
#define MOTOR_OFFSET 0                             //Factor to account for one side being more powerful
float timeOut = 2*(MAX_DIST+10)/100/340*1000000;    //Maximum time to wait for a return signal
unsigned long lastTime;                             // Define timer
bool isIrHight;                                     // Define Infrared state
int turnDuration =  round(35000/(MOTOR_SPEED+TURN_SPEED)); // Calculate 90 deg turn time 

void setup() 
{
  setSpeed(false);
  stopMove();
  servoLook.attach(SERVO);                           //Assign the servo pin
  pinMode(TRIG,OUTPUT);                           //Assign ultrasonic sensor pin modes
  pinMode(ECHO,INPUT);
  pinMode(IR_LEFT, INPUT);                        // Infrared input
  pinMode(IR_RIGHT, INPUT);
}

void loop() 
{
  servoLook.write(90);                            //Set the servo to look straight ahead
  delay(550);
  int distance = getDistance();                   //Check that there are no objects ahead
  isIrHight = digitalRead( IR_LEFT ) == HIGH && digitalRead( IR_RIGHT ) == HIGH;
  if(distance >= STOP_DIST && isIrHight)                        //If there are no objects within the stopping distance, move forward
  {
    moveForward();
  }
  while(distance >= STOP_DIST && isIrHight)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    isIrHight = digitalRead( IR_LEFT ) == HIGH && digitalRead( IR_RIGHT ) == HIGH;
    distance = getDistance();
  }
  stopMove();                                     //Stop the motors
  int turnDir = checkDirection();                 //Check the left and right object distances and get the turning instruction
  switch (turnDir)                                //Turn left, turn around or turn right depending on the instruction
  {
    case 0:                                       //Turn left
      turnLeft (turnDuration);
      break;
    case 1:                                       //Turn around
      turnLeft (turnDuration * 2);
      break;
    case 2:                                       //Turn right
      turnRight (turnDuration);
      break;
  }
}

void accelerate()                                 //Function to accelerate the motors from 0 to full speed
{
  for (int i=0; i<MOTOR_SPEED; i++)                //Loop from 0 to full speed
  {
    rightBack.setSpeed(i);                        //Set the motors to the current loop speed
    rightFront.setSpeed(i);
    leftFront.setSpeed(i+MOTOR_OFFSET);
    leftBack.setSpeed(i+MOTOR_OFFSET);
    delay(10);
  }
}

void decelerate()                                 //Function to decelerate the motors from full speed to zero
{
  for (int i=MOTOR_SPEED; i!=0; i--)               //Loop from full speed to 0
  {
    rightBack.setSpeed(i);                        //Set the motors to the current loop speed
    rightFront.setSpeed(i);
    leftFront.setSpeed(i+MOTOR_OFFSET);
    leftBack.setSpeed(i+MOTOR_OFFSET); 
    delay(10);
  }
}

void moveForward()                                //Set all motors to run forward
{
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove()                                   //Set all motors to stop
{
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void setSpeed(bool isIncremental)                                   //Set the motors to the motor speed
{
  int turnSpeed = MOTOR_SPEED;
  if (isIncremental) {
    turnSpeed += TURN_SPEED;
  }
  rightBack.setSpeed(turnSpeed);                           
  rightFront.setSpeed(turnSpeed);
  leftFront.setSpeed(turnSpeed+MOTOR_OFFSET);
  leftBack.setSpeed(turnSpeed+MOTOR_OFFSET);
}

void turnDirection(bool isRight) 
{
  int leftRotation, rightRotation;
  if (isRight) {
    leftRotation = FORWARD;
    rightRotation = BACKWARD;
  } else {
    leftRotation = BACKWARD;
    rightRotation = FORWARD;
  }
  rightBack.run(rightRotation);
  rightFront.run(rightRotation);
  leftFront.run(leftRotation);
  leftBack.run(leftRotation);
}

void turnLeft(int duration)                                 //Set motors to turn left for the specified duration then stop
{
  setSpeed(true);
  turnDirection(false);
  delay(duration);
  setSpeed(false);
  stopMove();
  
}

void turnRight(int duration)                                //Set motors to turn right for the specified duration then stop
{
  setSpeed(true);
  turnDirection(true);
  delay(duration);
  setSpeed(false);
  stopMove();
}

int getDistance()                                   //Measure the distance to an object
{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(TRIG, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  pulseTime = pulseIn(ECHO, HIGH, timeOut);         //Measure the time for the pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object distance based on the pulse time
  return distance;
}

int checkDirection()                                            //Check the left and right directions and decide which way to turn
{
  int distances [2] = {0,0};                                    //Left and right distances
  int turnDir = 1;                                              //Direction to turn, 0 left, 1 reverse, 2 right
  servoLook.write(180);                                         //Turn servo to look left
  delay(500);
  distances [0] = getDistance();                                //Get the left object distance
  servoLook.write(0);                                           //Turn servo to look right
  delay(500);
  distances [1] = getDistance();                                //Get the right object distance
  if (distances[0]>=MAX_DIST && distances[1]>=MAX_DIST)                   //If both directions are clear, turn left
    turnDir = 0;
  else if (distances[0]<=STOP_DIST && distances[1]<=STOP_DIST)    //If both directions are blocked, turn around
    turnDir = 1;
  else if (distances[0]>=distances[1])                          //If left has more space, turn left
    turnDir = 0;
  else if (distances[0]<distances[1])                           //If right has more space, turn right
    turnDir = 2;
  return turnDir;
}
