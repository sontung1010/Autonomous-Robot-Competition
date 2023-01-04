
// MD = MIDDLE ; 
String command; 
#define echoPinMD 7 // attach pin D9 Arduino to pin Echo of HC-SR04
#define trigPinMD 8 //attach pin D8 Arduino to pin Trig of HC-SR04
#define redLED 4

// defines variables
long durationMD; // variable for the duration of sound wave travel MIDDLE
int distanceMD; // variable for the distance measurement MIDDLE

void setup() {
  pinMode(trigPinMD, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPinMD, INPUT); // Sets the echoPin as an INPUT
  pinMode(redLED, OUTPUT);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
 // Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  //Serial.println("with Arduino UNO R3");
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPinMD, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPinMD, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinMD, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationMD = pulseIn(echoPinMD, HIGH);

  // Calculating the distance MIDDLE
  distanceMD = durationMD * 0.034 / 2; // Speed of sound wave divided by 2 (ARRIVAL and RETURN)
  // Calculating the distance DRIVERSIDE
  
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("stop")) {
      digitalWrite(redLED, HIGH);
    }
    else if (command.equals("move")){
      digitalWrite(redLED, LOW);
    }
  }
  
  // Displays the distance on the Serial Monitor
 // Serial.print("DISTANCE DETECTED FROM MIDDLE SENSOR: ");
  Serial.println(distanceMD);
 // Serial.println(" cm");
  delay(100);
}
