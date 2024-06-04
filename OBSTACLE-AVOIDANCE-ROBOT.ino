#include <AFMotor.h>
#include <Servo.h>

// Define ultrasonic sensor pins
int pingPinC = A0;
int echoPinC = A1;
int pingPinL = A2;
int echoPinL = A3;
int pingPinR = A4;
int echoPinR = A5;

// Define constants
#define MAX_DISTANCE 300
#define MAX_SPEED 200
#define MAX_SPEED_OFFSET 40
#define COLL_DIST 30

// Initialize motor objects
AF_DCMotor leftMotor1(1, MOTOR12_1KHZ);
AF_DCMotor leftMotor2(2, MOTOR12_1KHZ);
AF_DCMotor rightMotor1(3, MOTOR34_1KHZ);
AF_DCMotor rightMotor2(4, MOTOR34_1KHZ);

// Declare variables
float leftDistance, rightDistance, curDist;
String motorSet = "";
int speedSet = 160;
unsigned long previousMillis = 0;

//----------------------------------------------------- PING FUNCTIONS------------------------------------------------------

// Function to trigger the ultrasonic sensor and calculate distance
void triggerPing(int pingPin, int echoPin, float &distance)
{
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2.0) * 0.0343;
}

// Function to read distances from all three ultrasonic sensors
void readPing()
{
    triggerPing(pingPinC, echoPinC, curDist);
    Serial.print("curDist : ");
    Serial.println(curDist);
    triggerPing(pingPinR, echoPinR, rightDistance);
    Serial.print("rightDistance : ");
    Serial.println(rightDistance);
    triggerPing(pingPinL, echoPinL, leftDistance);
    Serial.print("leftDistance : ");
    Serial.println(leftDistance);
}

//--------------------------------------------------------MOVE FUNCTIONS--------------------------------------------------

// Function to stop all motors
void moveStop()
{
    Serial.println("Moving Stop :");
    leftMotor1.run(RELEASE);
    leftMotor2.run(RELEASE);
    rightMotor1.run(RELEASE);
    rightMotor2.run(RELEASE);
}

// Function to move the robot forward
void moveForward()
{
    Serial.println("Moving Forward :");
    motorSet = "FORWARD";
    leftMotor1.run(FORWARD);
    leftMotor2.run(FORWARD);
    rightMotor1.run(FORWARD);
    rightMotor2.run(FORWARD);
}

// Function to move the robot backward
void moveBackward()
{
    Serial.println("Moving Backward :");
    motorSet = "BACKWARD";
    leftMotor1.run(BACKWARD);
    leftMotor2.run(BACKWARD);
    rightMotor1.run(BACKWARD);
    rightMotor2.run(BACKWARD);
}

// Function to turn the robot right
void turnRight()
{
    motorSet = "RIGHT";
    leftMotor1.run(FORWARD);
    leftMotor2.run(FORWARD);
    rightMotor1.run(BACKWARD);
    rightMotor2.run(BACKWARD);
    Serial.println("Moving Right :");
    delay(280);
    leftMotor1.run(RELEASE);
    leftMotor2.run(RELEASE);
    rightMotor1.run(RELEASE);
    rightMotor2.run(RELEASE);
    delay(300);
}

// Function to turn the robot left
void turnLeft()
{
    motorSet = "LEFT";
    leftMotor1.run(BACKWARD);
    leftMotor2.run(BACKWARD);
    rightMotor1.run(FORWARD);
    rightMotor2.run(FORWARD);
    Serial.println("Moving Left :");
    delay(280);
    leftMotor1.run(RELEASE);
    leftMotor2.run(RELEASE);
    rightMotor1.run(RELEASE);
    rightMotor2.run(RELEASE);
    delay(280);
}

// Function to perform a 180-degree turn
void turnAround()
{
    motorSet = "LEFT";
    leftMotor1.run(BACKWARD);
    leftMotor2.run(BACKWARD);
    rightMotor1.run(FORWARD);
    rightMotor2.run(FORWARD);
    delay(560);
    motorSet = "FORWARD";
    leftMotor1.run(RELEASE);
    leftMotor2.run(RELEASE);
    rightMotor1.run(RELEASE);
    rightMotor2.run(RELEASE);
    delay(300);
}

//-------------------------------------------- SETUP LOOP ---------------------------------------------------------------------

void setup()
{
    // Initialize servo and serial communication
    myservo.attach(9);
    myservo.write(90);
    Serial.begin(9600);

    // Set pin modes for ultrasonic sensors
    pinMode(pingPinC, OUTPUT);
    pinMode(echoPinC, INPUT);
    pinMode(pingPinR, OUTPUT);
    pinMode(echoPinR, INPUT);
    pinMode(pingPinL, OUTPUT);
    pinMode(echoPinL, INPUT);

    // Set initial speed for all motors
    leftMotor1.setSpeed(speedSet);
    leftMotor2.setSpeed(speedSet);
    rightMotor1.setSpeed(speedSet);
    rightMotor2.setSpeed(speedSet);
}

//---------------------------------------------MAIN LOOP ----------------------------------------------------------------------
void loop()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 50)
    {
        previousMillis = currentMillis;
        readPing();
        if (curDist > COLL_DIST && rightDistance > 20 && leftDistance > 20)
        {
            while (curDist > COLL_DIST && rightDistance > 20 && leftDistance > 20)
            {
                readPing();
                moveForward();
            }
            moveStop();
            readPing();
        }
        else if (curDist < COLL_DIST && rightDistance < 20 && leftDistance < 20)
        {
            turnAround();
            readPing();
        }
        else if (rightDistance > leftDistance)
        {
            turnRight();
            readPing();
        }
        else if (rightDistance < leftDistance)
        {
            turnLeft();
            readPing();
        }
        else if (rightDistance == leftDistance)
        {
            turnAround();
            readPing();
        }
        else
        {
            turnAround();
            readPing();
        }
    }
}
