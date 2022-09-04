#include <QTRSensors.h>
#include <Encoder.h>
#include <Servo.h>

#define SERVO_PIN  10

#define CW   1900
#define STOP 1450
#define CCW  1000

Servo myServo;

Encoder myEnc1(21, 20);
Encoder myEnc2(18, 19);

#define TPR 374 //Ticks Per Round
#define whiteDist 280 //15cm of ticks

#define Kp 0.01//0.09 //0.68
// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.04//0.175//0.42 //5
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)  2.3

#define speedturn 180 //100
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define rightMaxSpeed 140 // max speed of the robot
#define leftMaxSpeed 140 // max speed of the robot
#define rightBaseSpeed 110 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 110 // this is the speed at which the motors should spin when the robot is perfectly on the line

QTRSensorsRC qtrrc((unsigned char[]) {
  A15, A14, A13, A12, A11, A10, A9, A8
} , NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

unsigned int SensorValues[8];
unsigned int sensors [8];


#define ena 8
#define enb 9

#define in4 A3
#define in3 A4
#define in1 A5
#define in2 A6

#define sigCount 3

int taskCount;

long oldPos1;
long oldPos2;

long newPos1;
long newPos2;

int signals[sigCount];
int sigPins[sigCount];

void setup() {
  sigPins[0] = 20;
  sigPins[1] = 18;
  sigPins[2] = 16;
  
  //encoders setup
  oldPos1 = -999;
  oldPos2 = -999;

  //servo setup
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(STOP); 

  //serial setup
  Serial.begin(9600);

  //pin setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  //calibrating qtr
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();


  for (int i = 0 ; i < 150; i++){
    qtrrc.calibrate();
    delay(20);
  }

  taskCount = 0;

}

void loop() {
  if(taskCount == 0){
    start();
  }else if(taskCount == 1){
    waitPass();
  }else if(taskCount == 2){
    firstblue();
  }else if(taskCount == 3){
    firstLeft();
  }else if(taskCount == 4){
    secondLeft();
  }else if(taskCount == 5){
    secondBlue();
  }else if(taskCount == 6){
    firstRight();
  }else if(taskCount == 7){
    thirdBlue();
  }else if(taskCount == 8){
    unload();
  }else if(taskCount == 9){
    thirdLeft();
  }else if(taskCount == 10){
    fourthLeft();
  }else if(taskCount == 11){
    fifthLeft();
  }else if(taskCount == 12){
    waitCar();
  }else if(taskCount == 13){
    lastLeft();
  }else if(taskCount == 14){
    finish();
  }
}

void  forwardPID(){
  int position = qtrrc.readLine(SensorValues);
  int error, lastError;
  error = position - 3500 ;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  
  int rightMotorSpeed = rightBaseSpeed + motorSpeed ;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ){
    rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  } else if (rightMotorSpeed < 0){
    rightMotorSpeed = 0; // keep the motor speed positive
  }
  if (leftMotorSpeed > leftMaxSpeed ){
    leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  }else if (leftMotorSpeed < 0){
    leftMotorSpeed = 0; // keep the motor speed positive
  }
  
  analogWrite(ena,rightMotorSpeed);
  analogWrite(enb,leftMotorSpeed);
  forward();
}

void start(){
  readSigs();
  if(signals[0] == 1){
    stp();
    signals[0] = 0;
    openContainer(200);
    delay(7000);
    closeContainer(200);
    taskCount++;
  }else{
    forwardPID();
  }
}

void waitPass(){
  bool cond;
  if(cond){
    stp();
    delay(3000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void firstblue(){
  readSigs();
  if(signals[1] == 1){
    stp();
    signals[1] = 0;
    delay(2000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void firstLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void secondLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void secondBlue(){
  readSigs();
  if(signals[1] == 1){
    stp();
    signals[1] = 0;
    delay(2000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void firstRight(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void thirdBlue(){
  readSigs();
  if(signals[1] == 1){
    stp();
    signals[1] = 0;
    delay(2000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void unload(){
  readSigs();
  if(signals[2] == 1){
    stp();
    signals[2] = 0;
    openContainer(1000);
    delay(6000);
    closeContainer(1000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void thirdLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void fourthLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void fifthLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void waitCar(){
  bool cond;
  if(cond){
    stp();
    delay(3000);
    taskCount++;
  }else{
    forwardPID();
  }
}


void lastLeft(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}

void finish(){
  bool cond;
  if(cond){
    stp();
    delay(30000);
  }else{
    forwardPID();
  }
}


void openContainer(int deg){
  myServo.writeMicroseconds(CW);
  delay(deg);
}

void closeContainer(int deg){
  myServo.writeMicroseconds(CCW);
  delay(deg);
}

bool checkSig(int pin){
  int a = digitalRead(pin);
  delay(1);
  int b = digitalRead(pin);
  delay(1);
  int c = digitalRead(pin);
  delay(1);
  int d = digitalRead(pin);
  if(a&&b&&c&&d){
    return true;
  }else{
    return false;
  }
}

int readOneSig(int pin){
  if(digitalRead(pin)){
    bool ok = checkSig(pin);
    if (ok){
      return 1;
    }
  }
  return 0;
}

void readSigs(){
  for(int i = 0; i < sigCount; i++){
    signals[i] = readOneSig(sigPins[i]);
  }
}


void forward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}
void wa55er(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH);
}
void left(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, LOW);
}

void right(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, HIGH);
}
void lefts(){
  digitalWrite(in1, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}

void rights(){
  digitalWrite(in1, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}

void stp(){
  analogWrite(ena,0);
  analogWrite(enb,0);
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}
void stpDelay(int a){
  analogWrite(ena,0);
  analogWrite(enb,0);
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
  delay(a);
}
