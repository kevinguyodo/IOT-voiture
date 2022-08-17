#include "Freenove_WS2812B_RGBLED_Controller.h";
#define I2C_ADDRESS  0x20
#define LEDS_COUNT   10  //it defines number of lEDs. 

#include <Servo.h>      //servo liberary

#define servoPin  2    //define servo pin

#define echoPin 8 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 7 //attach pin D3 Arduino to pin Trig of HC-SR04

#define buzzerPin A0

#define rightDirection 3
#define leftDirection 4
#define motorRightPin 5
#define motorLeftPin 6

#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)

Freenove_WS2812B_Controller strip(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB); //initialization

Servo servo;            //create servo object to control a servo
int speedOffset;

const int alert = 25;

const int toClose = 25; // Obstacle too close
const int obstacle = 50; // Obstacle distance

void setup() 
{      
    // Initialisation HC-SR04
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(rightDirection, OUTPUT);
    pinMode(leftDirection, OUTPUT);
    pinMode(motorRightPin, OUTPUT);
    pinMode(motorLeftPin, OUTPUT);

    // Init interface série 
    Serial.begin(9600); 
  

    servo.attach(servoPin);
    
    delay(1000);
}

void loop() 
{      
  while (!strip.begin());
  delay(100);
  avoidAnObstacle();
}


int getProximity() {
  long duration;
  int distance; 
    // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void changeLedColor(int proximity) {
  int ledOffOn = (proximity*alert);
  if (proximity < 25) {
    strip.setAllLedsColor(0xFF0000);
    delay(ledOffOn);
    strip.setAllLedsColor(0,0,0);
    delay(ledOffOn); 
    //toCloseReaction(proximity, ledOffOn);
  } else if (proximity < 60 && proximity >= 25) {
    strip.setAllLedsColor(255, 255, 0); //set all LED color to yellow. this is just deffent form of rgb value. 
    delay(ledOffOn);
    strip.setAllLedsColor(0,0,0);
    delay(ledOffOn);
  } else {
    strip.setAllLedsColor(0x00FF00); //set all LED color to green
  }
}


void runCar(int speedl, int speedr) {
  int dirL = 0;
  int dirR = 0;
  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }
  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }

  digitalWrite(leftDirection, dirL);
  digitalWrite(rightDirection, dirR); 
  analogWrite(motorLeftPin, speedl);
  analogWrite(motorRightPin, speedr);
}

void avoidAnObstacle() {
  int distance[3], tempDistance[3][5], sumDisntance;
  static u8 leftToRight = 0, servoAngle = 0, lastServoAngle = 0;  
  const u8 scanAngle[2][3] = { {150, 90, 30}, {30, 90, 150} };

  for (int i = 0; i < 3; i++)
  {
    servoAngle = scanAngle[leftToRight][i];
    servo.write(servoAngle);
    if (lastServoAngle != servoAngle) {
      delay(130);
    }
    lastServoAngle = servoAngle;
    for (int j = 0; j < 5; j++) {
      tempDistance[i][j] = getProximity();
      delayMicroseconds(2 * SONIC_TIMEOUT);
      sumDisntance += tempDistance[i][j];
    }
    if (leftToRight == 0) {
      distance[i] = sumDisntance / 5;
    }
    else {
      distance[2 - i] = sumDisntance / 5;
    }
    sumDisntance = 0;
  }
  leftToRight = (leftToRight + 1) % 2;

  if (distance[1] < obstacle) {        //Too little distance ahead
    changeLedColor(distance[1]);
    if (distance[0] > distance[2] && distance[0] > obstacle) {     //Left distance is greater than right distance
      runCar(-(150 + speedOffset), -(150 + speedOffset)); //Move back
      delay(100);
      runCar(-(150 + speedOffset), (150 + speedOffset));  
    }
    else if (distance[0] < distance[2] && distance[2] > obstacle) {                   //Right distance is greater than left distance
      runCar(-(150 + speedOffset), -(150 + speedOffset)); //Move back 
      delay(100);
      runCar((150 + speedOffset), -(150 + speedOffset));
    }
    else {                      //Get into the dead corner, move back, then turn.
      runCar(-(150 + speedOffset), -(150 + speedOffset));
      delay(100);
      runCar(-(150 + speedOffset), (150 + speedOffset));
    }
  }
  else {                        //No obstacles ahead
    changeLedColor(distance[1]);
    if (distance[0] <  toClose) {      //Obstacles on the left front.
      runCar(-(150 + speedOffset), -(150 + speedOffset)); //Move back
      delay(100);
      runCar((180 + speedOffset), (50 + speedOffset));
    }
    else if (distance[2] <  toClose) {     //Obstacles on the right front.
      runCar(-(150 + speedOffset), -(150 + speedOffset)); //Move back
      delay(100);
      runCar((50 + speedOffset), (180 + speedOffset));
    }
    else {                        //Cruising
      runCar((80 + speedOffset), (80 + speedOffset));
    }
  }
}
