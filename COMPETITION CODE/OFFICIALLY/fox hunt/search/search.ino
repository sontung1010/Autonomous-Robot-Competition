//Sensor1
//MD = MIDDLE ; 
String command; 
#define echo1 PD3 // attach pin D9 Arduino to pin Echo of HC-SR04
#define trig1 PF5 //attach pin D8 Arduino to pin Trig of HC-SR04
#define echo2 PB11 // 
#define trig2 PE0
#define echo3 PA0 // 
#define trig3 PE15

#define echo4 PE2 // 
#define trig4 PE12

#define echo5 PD11 // 
#define trig5 PE10

#define redLED PB0  //YELLOW
#define LED2 PB14 //RED
#define LED3 PB7 //BLUE


long duration, distance, RightSensor,BackSensor,FrontSensor_Mid,FrontSensor_Left,FrontSensor_Right,LeftSensor;

int speed = 200;
int speed_a = 50;
int backspeed = 70;
int turnspeed = 180;
int a=0;

int diff = 0;
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
  // put your setup code here, to run once:
  pinMode(trig1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo1, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo2, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig3, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo3, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig4, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo4, INPUT); // Sets the echoPin as an INPUT
  pinMode(trig5, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echo5, INPUT); // Sets the echoPin as an INPUT

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


SonarSensor(trig1, echo1);
FrontSensor_Mid = distance;
SonarSensor(trig2, echo2);
RightSensor = distance;
SonarSensor(trig3, echo3);
LeftSensor = distance;
SonarSensor(trig4, echo4);
FrontSensor_Left = distance;
SonarSensor(trig5, echo5);
FrontSensor_Right = distance;

diff = abs(RightSensor - LeftSensor);


Serial.print(" SensorFrontmid ");
Serial.print(FrontSensor_Mid);

Serial.print(" SensorFrontleft ");
Serial.print(FrontSensor_Left);

Serial.print(" SensorFrontright ");
Serial.print(FrontSensor_Right);

Serial.print(" SensorRight ");
Serial.print(RightSensor);

Serial.print("  SensorLeft ");
Serial.println(LeftSensor);
delayMicroseconds(1000);

while(a<60000){
  moveForward();
//Rotate();
  a++;  
}
while(a >= 60000) {
turnLeft();
delay(1000);
}


/*while(a>35000 && a<50000){
  turnLeft();
  Rotate();
  a++;  
}
 
  while(a<750000 && a>=500000){
      moveBackward();

 // moveBackward();
  a++;  
}
*/
  moveBackward_2();
  delayMicroseconds(150);
  stopMoving();
  delayMicroseconds(8000);
  /*
  if(FrontSensor_Mid < 50 || FrontSensor_Left < 50 || FrontSensor_Right <50){
    
    moveBackward();
    delayMicroseconds(50);
  }
  else{
    digitalWrite(LED2, LOW);
    if (diff > 5){

    }
    if (RightSensor<LeftSensor){
      turnLeft(); 
      delayMicroseconds(100);
    }
    else{
      turnRight();
      delayMicroseconds(100);
      */
    

 // delayMicroseconds(5000);
 /// stopMoving();
 // delayMicroseconds(100000);
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
  analogWrite(PWM_FLM,speed+6); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,speed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,speed+6); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,speed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void moveBackward() {
  analogWrite(PWM_FLM,speed_a+8); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,speed_a); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,speed_a+8); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,speed_a); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}
void moveBackward_2() {
  analogWrite(PWM_FLM,backspeed); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,backspeed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,backspeed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,backspeed); //ENA pin
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

  analogWrite(PWM_FRM,0); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,turnspeed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,0); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}
void Rotate() {
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
  analogWrite(PWM_FLM,0); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,turnspeed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,0); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,turnspeed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);

}