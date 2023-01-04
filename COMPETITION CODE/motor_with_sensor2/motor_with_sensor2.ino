#include <NewPing.h>

#define MAX_DISTANCE 200

#define SONAR_NUM 3

//Sensor1
//MD = MIDDLE ; 
String command; 

#define redLED PB0  //YELLOW
#define LED2 PB14 //RED
#define LED3 PB7 //BLUE

NewPing sonar[SONAR_NUM] = {
  NewPing(PD3, PF5, MAX_DISTANCE),
  NewPing(PB11, PE0, MAX_DISTANCE),
  NewPing(PA0, PE15, MAX_DISTANCE)
}


long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;

int speed = 70;
int turnspeed = 70;
//FLM
int motor1pin1 = PG14;
int motor1pin2 = PF15;
int PWM_FLM = PE13;

int motor2pin1 = PF14;
int motor2pin2 = PE11;
int PWM_FRM = PE9;

int motor3pin1 = PF13;
int motor3pin2 = PF12;
int PWM_RLM = PD15;

int motor4pin1 = PD14;
int motor4pin2 = PA7;
int PWM_RRM = PA6;

void setup() {

  pinMode(trig1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo1, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo2, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig3, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo3, INPUT); // Sets the echoPin as an INPUT

  pinMode(redLED, OUTPUT);
  
  pinMode(LED2, OUTPUT);
  
  pinMode(LED3, OUTPUT);
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);

  pinMode(PWM_FLM, OUTPUT); 

  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(PWM_FRM, OUTPUT); 

  pinMode(motor3pin1, OUTPUT);
  pinMode(motor3pin2, OUTPUT);

  pinMode(PWM_RLM, OUTPUT); 

  pinMode(motor4pin1, OUTPUT);
  pinMode(motor4pin2, OUTPUT);

  pinMode(PWM_RRM, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:

   // Clears the trigPin condition
  
SonarSensor(trig1, echo1);
FrontSensor = distance;
SonarSensor(trig2, echo2);
RightSensor = distance;
SonarSensor(trig3, echo3);
LeftSensor = distance;

Serial.print(" SensorFront ");
Serial.print(FrontSensor);

Serial.print(" SensorRight ");
Serial.print(RightSensor);

Serial.print("  SensorLeft ");
Serial.println(LeftSensor);


  if 

   /*if(distance1 <40)
  {
    stopMoving();
  }
  else
  {
    turnLeft();
  }
  */
}

void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;

}
void moveForward() {
  analogWrite(PWM_FLM,speed); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,speed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,speed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,speed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void moveBackward() {
  analogWrite(PWM_FLM,speed); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,speed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,speed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,speed); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}
void stopMoving() {
  analogWrite(PWM_FLM,0); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,0); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,0); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,0); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void turnRight() {
  analogWrite(PWM_FLM,turnspeed); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,turnspeed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,turnspeed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,turnspeed); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void turnLeft() {
  analogWrite(PWM_FLM,turnspeed); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,turnspeed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,turnspeed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,turnspeed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}