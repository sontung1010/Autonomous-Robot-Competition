// MD = MIDDLE ; 

#define echoPinMD PD3 // attach pin D3 Arduino to pin Echo of HC-SR04
#define trigPinMD PF5 //attach pin F5 Arduino to pin Trig of HC-SR04
#define echoPinML PE2
#define trigPinML PE12
#define echoPinMR PD11
#define trigPinMR PE10
#define echoPinR PB11 // attach pin B11 Arduino to pin Echo of HC-SR04
#define trigPinR PE0 //attach pin E0 Arduino to pin Trig of HC-SR04
#define echoPinL PA0 // attach pin A0 Arduino to pin Echo of HC-SR04
#define trigPinL PE15 //attach pin E15 Arduino to pin Trig of HC-SR04

#define LED1 PB0
#define LED2 PB7


// defines variables
long durationMD; // variable for the duration of sound wave travel MIDDLE
int distanceMD; // variable for the distance measurement MIDDLE
const int SensorNumber = 5;
int mapCounter = 0;
int SensorMap[SensorNumber][250];      // Newest map0
int SensorMap1[SensorNumber][250];     // Old map1
int SensorMap2[SensorNumber][250];     // Oold map2
int SensorMap3[SensorNumber][250];     // Filtered map

int speedLow = 80;
int speed = 120;
int speedHigh = 140;
int turnspeed = 140;
float reduceFactor = 0.6;
int brakeingDistance = 60;
int slowspeedDistance = 100;
int midspeedDistance = 150;

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
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}

void loop() {
  (mapCounter < 3)?(mapCounter++):(mapCounter = 1);
  Scan();                             // Scan the surrounding and crate a map
  Fmap();                             // Filter the map
  fallow();                            // Avoid base on the map
  //for( int i = 0; i < SensorNumber; i++)
    //DisplayMap(i);
  Serial.print("\n");
  
}

int fallow(){
  int direction = 99, direction1 = 99, direction2 = 99, distance = 0, distance1 = 0, distance2 = 0;
  for(int i = 0; i < 200; i ++){      // Search for the 1 'object' that is closest to the robot
    for(int j = 0; j < SensorNumber; j++){
      if((SensorMap3[j][i] == 1) && (direction == 99)){       // The first 1 'object' it found
        direction = j;                // set the direction
        distance = i;                 // set the distance
        j = SensorNumber;                        // Exit for loop
        i = 250;
        Serial.print(distance);
      }
      /*if((SensorMap3[j][i] == 1) && (direction != j) && (direction != 99) && (direction1 == 99)){
        // The second 1 'object' it found
        direction = j;                // set the direction
        distance1 = i;                 // set the distance
      }
      if((SensorMap3[j][i] == 1) && (direction != j) && (direction1 != j) && (direction1 != 99) && (direction2 == 99)){
        direction = j;                // set the direction
        distance2 = i;                 // set the distance
      }*/
    } 
  }
  char action;
  int fallowSpeed = speedHigh;              // set default at full speed
  
  
  switch(direction){
    case 0:
      action = 'L';   // hard left
      break;
    case 1:
      action = 'l';   // left
      break;
    case 2:
      action = 'f';   // front
      break;
    case 3:
      action = 'r';   // right
      break;
    case 4:
      action = 'R';   // hard right
      break;
  }

 /* if(distance < 100 ){
    fallowSpeed = speed;                  // if object is in less than 200 unit, reduce speed
    if(distance < 50){
      fallowSpeed = speedLow;             // if object is in less than 100 unit, reduce more speed
      if(distance < 10){
        action = 'S';
        fallowSpeed = 0;                  // if object is in less than 50 unit, stop
      }
    }
  }*/
  (distance < midspeedDistance)?(fallowSpeed = speed):(fallowSpeed = fallowSpeed);       // if object is in less than 60 unit, reduce speed
  (distance < slowspeedDistance)?(fallowSpeed = speedLow):(fallowSpeed = fallowSpeed);    // if object is in less than 40 unit, reduce more speed
  (distance < brakeingDistance)?(action = 'S'):(fallowSpeed = fallowSpeed);          // if object is in less than 10 unit, stop

  /*if(distance < 50){
    action = 'S';             // stop moving
  }
  if(((direction1 == (direction+1)) || (direction1 == (direction-1)))  && ((direction2 == (direction1+1)) || (direction2 == (direction1-1)))){
    // all three detected sensors are adjacent
    if(distance2 < (distance+10)){         // object3 three is near object2
      if(distance3 < (distance2+10)){       // Assume threee sensor sense the same thing
        a
      }
    }
  }
  else if((direction1 == (direction+1)) || (direction1 == (direction-1))){
    // the closest two are adjacent
  }
  else{
    //the closest 
  }
  if(distance2 < (distance+10)){         // Assume two sensor sense the same thing
    if(distance3 < (distance2+10)){       // Assume threee sensor sense the same thing
      a
    }
  }*/
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  Display_Direction(action, fallowSpeed);
  return direction;
}

void Display_Direction(char action, int fallowSpeed){
  switch(action){
    case 'L':
      turnLeft();
      Serial.print("Left\n");
      break;
    case 'l':
      turnlLeftForward(fallowSpeed);
      Serial.print("Left front\n");
      break;
    case 'f':
      moveForward(fallowSpeed);
      Serial.print("front\n");
      break;
    case 'r':
      turnRightForward(fallowSpeed);
      Serial.print("Right front\n");
      break;
    case 'R':
      turnRight();
      Serial.print("Right\n");
      break;      
    case 'S':
      stopMoving();
      Serial.print("Stop\n");
      break;
    default:
      moveForward(40);          // default slowly move forward
      Serial.print("Stop\n");
      break;
  }
}

// -------------------------- Ultrasonic Sensor ----------------------------------------------

void Scan (){
  for(int i = 0; i < SensorNumber; i ++){          // Loop through all ultrasonic sensors
    distanceMD = UltrasonicSelect(i);   // function return the distance the sensor scaned
    WriteMap(distanceMD,i);             // recorde the distance
    //DisplayMap(i);      
  }
}

int UltrasonicSelect (int SensorNumber){
  // Clears the trigPin condition
  switch(SensorNumber){
    case 0:
      return UltrasonicScan(echoPinL, trigPinL);    // front Left sensor
      break;
    case 1:
      return UltrasonicScan(echoPinML, trigPinML);      // front mid sensor
      break;
    case 2:
      return UltrasonicScan(echoPinMD, trigPinMD);      // front right sensor
      break;
    case 3:
      return UltrasonicScan(echoPinMR, trigPinMR);      // Left sensor
      break;
    case 4:
      return UltrasonicScan(echoPinR, trigPinR);      // right sensor
      break;
  }
}

int UltrasonicScan (int echo, int trig){
  int dis = 0;
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  // Reads the echoPin, returns the sound wave travel time in microseconds
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Calculating the distance MIDDLE
  // Speed of sound wave divided by 2 (ARRIVAL and RETURN)
  // Calculating the distance DRIVERSIDE
  durationMD = pulseIn(echo, HIGH);
  dis = durationMD * 0.034 / 2;
  return dis;
}

void WriteMap (int distance, int row){
  switch(mapCounter){                                 // Q maps
    case 1:
      for(int i = 0; i < 250; i++){                   // only record usable distance, 250 or closer
        (distance > i)?(SensorMap[row][i] = 0):(SensorMap[row][i] = 1);   // 0 is empty, 1 is object
      }
      break;
    case 2:
      for(int i = 0; i < 250; i++){                   // only record usable distance, 250 or closer
        (distance > i)?(SensorMap1[row][i] = 0):(SensorMap1[row][i] = 1);   // 0 is empty, 1 is object
      }
      break;
    case 3:
      for(int i = 0; i < 250; i++){                   // only record usable distance, 250 or closer
        (distance > i)?(SensorMap2[row][i] = 0):(SensorMap2[row][i] = 1);   // 0 is empty, 1 is object
      }
      break;    
    default:
      break;
  }
}

// --------------------------  Microphone Sensor -------------------------------------------------


//--------------------------------- map --------------------------------

void DisplayMap (int row){
  for(int i = 0; i < 90; i++){
    Serial.print(SensorMap3[row][i]);
    Serial.print(" ");
  }
  Serial.print("\n");
}

void Fmap(){
  for(int i = 0; i < SensorNumber; i ++){     // compare 3 maps to filter out the outlier 
    for(int j = 0; j < 250; j++){
      if(SensorMap[i][j] == SensorMap1[i][j]){        // map0 with map1
        SensorMap3[i][j] = SensorMap[i][j];
      }
      else if(SensorMap[i][j] == SensorMap2[i][j]){   // map0 with map2
        SensorMap3[i][j] = SensorMap[i][j];
      }
      else if(SensorMap1[i][j] == SensorMap2[i][j]){  // map1 with map2
        SensorMap3[i][j] = SensorMap1[i][j];
      }
      else{                                           // trust map0
        SensorMap3[i][j] = SensorMap[i][j];
      }
    }
  } 
}

//--------------------------------- motion ------------------------------

void moveForward(int fallowSpeed) {
  analogWrite(PWM_FLM,fallowSpeed); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,fallowSpeed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,fallowSpeed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,fallowSpeed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void moveBackward(int fallowSpeed) {
  analogWrite(PWM_FLM,fallowSpeed); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,fallowSpeed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,fallowSpeed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,fallowSpeed); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void stopMoving() {
  analogWrite(PWM_FLM,0); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,0); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,0); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,0); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, LOW);
}

// ----------------------------------------------------- Right ---------------------------------------------

void turnRight() {                // Outter motor at turn speed, inner motor at 0
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

void RightBack() {                // Outter motor at turn speed, inner motor at 0
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

void turnRightForward(int fallowSpeed) {      // Outter motor at fallow speed, inner motor at reduced fallowspeed
  int innerSpeed = reduceFactor * fallowSpeed;
  analogWrite(PWM_FLM,fallowSpeed); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,innerSpeed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,fallowSpeed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,innerSpeed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void turnRightBack(int fallowSpeed) {         // Outter motor at fallow speed, inner motor at reduced fallowspeed
  int innerSpeed = reduceFactor * fallowSpeed;
  analogWrite(PWM_FLM,fallowSpeed); 
  analogWrite(PWM_FLM,speedHigh); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,innerSpeed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,fallowSpeed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,innerSpeed); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

// ----------------------------------------------------- Left ---------------------------------------------

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

void LeftBack() {
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

void turnlLeftForward(int fallowSpeed) {            // Outter motor at fallow speed, inner motor at reduced fallowspeed
  int innerSpeed = reduceFactor * fallowSpeed;
  analogWrite(PWM_FLM,innerSpeed); //ENA pin
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(PWM_FRM,fallowSpeed); //ENA pin
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(PWM_RLM,innerSpeed); //ENA pin
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(PWM_RRM,fallowSpeed); //ENA pin
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void turnlLeftBack(int fallowSpeed) {            // Outter motor at fallow speed, inner motor at reduced fallowspeed
  int innerSpeed = reduceFactor * fallowSpeed;
  analogWrite(PWM_FLM,innerSpeed); //ENA pin
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(PWM_FRM,fallowSpeed); //ENA pin
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(PWM_RLM,innerSpeed); //ENA pin
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(PWM_RRM,fallowSpeed); //ENA pin
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}