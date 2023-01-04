// MD = MIDDLE ; 

#define echoPinMD PD3 // attach pin D3 Arduino to pin Echo of HC-SR04
#define trigPinMD PF5 //attach pin F5 Arduino to pin Trig of HC-SR04
#define echoPinML PE2
#define trigPinML PE12
#define echoPinMR PD11
#define trigPinMR PE10
#define echoPinRR PB9
#define trigPinRR PB8
#define echoPinRRR PB15
#define trigPinRRR PC6
#define echoPinR PB11 // attach pin B11 Arduino to pin Echo of HC-SR04
#define trigPinR PE0 //attach pin E0 Arduino to pin Trig of HC-SR04
#define echoPinL PA0 // attach pin A0 Arduino to pin Echo of HC-SR04
#define trigPinL PE15 //attach pin E15 Arduino to pin Trig of HC-SR04

#define LED1 PB0
#define LED2 PB7



long duration, distance, RightSensor,BackSensor,FrontSensor_Mid,FrontSensor_Left,FrontSensor_Right,LeftSensor;

int speedLow = 90;
int speed = 70;
int speedHigh = 90;
int turnspeed = 90;

int flag = 0;

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
  pinMode(trigPinMD, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinMD, INPUT); // Sets the echoPin as an INPUT
  pinMode(echoPinML, INPUT);
  pinMode(trigPinML, OUTPUT);
  pinMode(echoPinMR, INPUT);
  pinMode(trigPinMR, OUTPUT);
  pinMode(echoPinRR, INPUT);
  pinMode(trigPinRR, OUTPUT);
  pinMode(echoPinRRR, INPUT);
  pinMode(trigPinRRR, OUTPUT);
  pinMode(trigPinR, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinR, INPUT); // Sets the echoPin as an INPUT
  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinL, INPUT); // Sets the echoPin as an INPUT

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

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop() {
  SonarSensor(trigPinRRR, echoPinRRR);
FrontSensor_Mid = distance;
Serial.print(" SensorFrontmid ");
Serial.print(FrontSensor_Mid);
moveForward();
if (FrontSensor_Mid<120){
  flag = 1;
  turnLeft();
delayMicroseconds(50);
}
if (flag = 1 && FrontSensor_Mid>150){
  turnRight();
delayMicroseconds(50);
}
  
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
  analogWrite(PWM_FLM,speed+7); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,speed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,speed+7); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,speed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
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
  analogWrite(PWM_FLM,turnspeed+10); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,turnspeed-10); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,turnspeed+10); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,turnspeed-10); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void turnLeft() {
  analogWrite(PWM_FLM,turnspeed-10); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,turnspeed+10); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,turnspeed-10); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,turnspeed+10); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);

}