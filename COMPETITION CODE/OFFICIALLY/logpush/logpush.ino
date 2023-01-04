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

int speed = 115;
int backspeed = 100;
int turnspeed = 150;

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


moveForward();  
  
}


void moveForward() {
  analogWrite(PWM_FLM,speed+30); //ENA pin
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

