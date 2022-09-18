#include <QTRSensors.h>
#include <Encoder.h>
#include <Servo.h>

#define SERVO_PIN  45




#define CW   1900
#define STOP 1450
#define CCW  1000

Servo myServo;

Encoder myEnc2(2, 18);
Encoder myEnc1(20, 21);

#define TPR 374 //Ticks Per Round
#define whiteDist 280 //15cm of ticks

#define Kp 0.012//0.09 //0.68
// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.07//0.175//0.42 //5
// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)  2.3

#define speedturn 180 //100
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define rightMaxSpeed 180 // max speed of the robot
#define leftMaxSpeed 180 // max speed of the robot
#define rightBaseSpeed 140 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 140 // this is the speed at which the motors should spin when the robot is perfectly on the line

QTRSensorsRC qtrrc((unsigned char[]) {
  A12, A10, A8, A6, A4, A2, A0, A1
} , NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

unsigned int SensorValues[8];
unsigned int sensors [8];


int enb=5;
int ena=9;
int in3=7;
int in4=3;
int in1=13;
int in2=11;

int trigRight = 29;
int trigMid = 26;
int trigLeft = 23;

int echoRight = 31;
int echoMid = 28;
int echoLeft = 25;

#define sigCount 3

int taskCount;

int startloop = 0;

long oldPos1;
long oldPos2;

long newPos1;
long newPos2;

int signals[sigCount];
int sigPins[sigCount];

void setup() {
  sigPins[0] = A13;
  sigPins[1] = A11;
  sigPins[2] = A9;
  
  //encoders setup
  oldPos1 = -999;
  oldPos2 = -999;

  //servo setup
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(STOP); 

  //serial setup
  Serial.begin(9600);

  //pin setup
  pinMode(sigPins[0], INPUT);
  pinMode(sigPins[1], INPUT);
  pinMode(sigPins[2], INPUT);

  pinMode(echoLeft, INPUT);
  pinMode(echoRight, INPUT);
  pinMode(echoMid, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(trigMid, OUTPUT);
  pinMode(trigRight, OUTPUT);
  
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
  /*if(SensorValues[7] > 400 && SensorValues[6] > 400 && SensorValues[5] > 400){
     WheelControl(1950,1,150);
     stp();
     delay(5000);
  }else{
    forwardPID();  
  }*/
 
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

void WheelControl(int ticks1,int ticks2, int sped){
  long Pos1 = myEnc1.read();
  long Pos2 = myEnc2.read();
  long currPos1 = Pos1;
  long currPos2 = Pos2;
  bool ok1 = false;
  bool ok2 = false;
  while(1){
    if(ticks1 == 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      ok1 = true;
    }else if(ticks1 > 0){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      if(currPos1 - Pos1 < ticks1){
        analogWrite(ena,sped);
      }else{
        analogWrite(ena,0);
        ok1 = true;
      }
    }else if (ticks1 < 0){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      if(currPos1 - Pos1 > ticks1){
        analogWrite(ena,sped);
      }else{
        analogWrite(ena,0);
        ok1 = true;
      }
    }

    if(ticks2 == 0){
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      ok2 = true;
    }else if(ticks2 > 0){
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      if(currPos2 - Pos2 < ticks2){
        analogWrite(enb,sped);
      }else{
        analogWrite(enb,0);
        ok2 = true;
      }
    }else if (ticks2 < 0){
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      if(currPos2 - Pos2 > ticks2){
        analogWrite(enb,sped);
      }else{
        analogWrite(enb,0);
        ok2 = true;
      }
    }
    if(ok1 && ok2){
      break;
    }
    if(!ok1){
      currPos1 = myEnc1.read();
    }
    if(!ok2){
      currPos2 = myEnc2.read();
    }
    Serial.print("ok1 ");
    Serial.print(ok1);
    Serial.print(" - ok2 ");
    Serial.println(ok2);
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
  if(startloop == 0){
    encPID(10500);
  }else{
    readSigs();
    if(signals[0] == 1){
      stp();
      signals[0] = 0;
      //openContainer(200);
      delay(7000);
      //closeContainer(200);
      encPID(1000);
      taskCount++;
    }else{
      forwardPID();
    }
  }
}

void waitPass(){
  if((getDist(trigLeft,echoLeft)> 2 && getDist(trigLeft,echoLeft)< 12) || (getDist(trigMid,echoMid)> 2 && getDist(trigMid,echoMid)< 12) || (getDist(trigRight,echoRight)> 2 && getDist(trigRight,echoRight)< 12)){
    stp();
    delay(3000);
    encPID(1000);
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
    encPID(1000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void firstLeft(){
  if(SensorValues[7] > 400 && SensorValues[6] > 400 && SensorValues[5] > 400){
    WheelControl(2500,-1,150);
    stp();
    delay(1000);
    encPID(1000);
    taskCount++;
  }else{
    forwardPID();
  }
}

void secondLeft(){
  if(SensorValues[7] > 400 && SensorValues[6] > 400 && SensorValues[5] > 400){
    WheelControl(2500,-1,150);
    stp();
    delay(1000);
    encPID(1000);
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
  if(SensorValues[0] > 400 && SensorValues[1] > 400 && SensorValues[2] > 400){
    WheelControl(-1,2500,150);
    stp();
    delay(1000);
    encPID(1000);
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

void encPID(int dist){
  int startpos1 = myEnc1.read();
  int startpos2 = myEnc2.read();
  int cpos1 = startpos1;
  int cpos2 = startpos2;
  while(1){
    if(cpos1 - startpos1 > dist || cpos2 - startpos2 > dist){
      startloop = 1;
      break;
    }else{
      cpos1 = myEnc1.read();
      cpos2 = myEnc2.read();
      forwardPID();
    }
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

int getDist(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2; 
}
