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

#define TPR 374

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

int taskCount;

long oldPos1;
long oldPos2;

long newPos1;
long newPos2;


void setup() {

  //encoders setup
  oldPos1 = -999;
  oldPos2 = -999;

  //servo setup
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(STOP); 
  /*
    myServo.writeMicroseconds(CW); for clockwise
    myServo.writeMicroseconds(CCW); for counter clockwise
  */
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


  for (int i = 0 ; i < 120; i++){
    qtrrc.calibrate();
    delay(20);
  }

  taskCount = 0;

}

void loop() {
  
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
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    forwardPID();
  }
}
void wait(){
  bool cond;
  if(cond){
    /**/
    taskCount++;
  }else{
    delay(1);
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

void setSig(int * s){
  *s = 1;
}

void resetSig(int * s){
  *s = 0;
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
