// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;

    // derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }

    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }

    // store previous error
    eprev = e;
  }

};

// How many motors
#define NMOTORS 4

// Pins
const int enca[] = {8,9,10,11};
const int encb[] = {2,3,4,5};
const int pwm1 = PE9;
const int pwm2 = PE9;
const int pwm3 = PE9;
const int pwm4 = PE9;
const int in11 = PE10;
const int in12 = PE10;
const int in13 = PE10;
const int in14 = PE10;
const int in21 = PE11;
const int in22 = PE11;
const int in23 = PE11;
const int in24 = PE11;

// Globals
long prevT = 0;
volatile int posi[] = {0,0,0,0};

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(9600);

    pinMode(pwm1,OUTPUT);
    pinMode(in11,OUTPUT);
    pinMode(in21,OUTPUT);
    
    pinMode(pwm2,OUTPUT);
    pinMode(in12,OUTPUT);
    pinMode(in22,OUTPUT);
    
    pinMode(pwm3,OUTPUT);
    pinMode(in13,OUTPUT);
    pinMode(in23,OUTPUT);
    
    pinMode(pwm4,OUTPUT);
    pinMode(in14,OUTPUT);
    pinMode(in24,OUTPUT);
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pid[k].setParams(1,0,0,255);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]),readEncoder<3>,RISING);

  Serial.println("target pos");
}

void loop() {

  // set target position
  int target[NMOTORS];
  target[0] = 750;
  target[1] = 750;
  target[2] = 750;
  target[3] = 750;
  

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on

  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm1,in11,in21);
    setMotor(dir,pwr,pwm2,in12,in22);
    setMotor(dir,pwr,pwm3,in13,in23);
    setMotor(dir,pwr,pwm4,in14,in24);
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}