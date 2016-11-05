//Basic code is from https://github.com/zaidpirwani/DiffMotorVelocityControl

#define DEBUG 0
#define MAGICADDRESS 7

#include <math.h>
#include <EEPROM.h>

// For encoders
#include <Encoder.h>
Encoder encL(3,5);
Encoder encR(2,4);
long encoderL=0;
long encoderR=0;

// For Motor Control
#include "MotorLib.h"
MotorLib motorL(6, 7, 9);
MotorLib motorR(12, 13, 10);
// Struct to hold motor parameters
struct motorParams {
  double kp;
  double ki;
  double kd;
};
motorParams motorPIDL;
motorParams motorPIDR;
motorParams rotatePID;

// For PID for motors
#include <PID_v1_my.h>
double vel1 = 0, vel2 = 0;
double spd1, spd2;
double pwm1, pwm2;
double currentH,setpoint,pwmRotate;
PID pidL(&spd1, &pwm1, &vel1, 1,0,0, DIRECT);
PID pidR(&spd2, &pwm2, &vel2, 1,0,0, DIRECT);
PID pidRotate(&currentH, &pwmRotate, &setpoint, 1,0,0, DIRECT);   //for rotate
boolean pidActiveS = false;   //for straight
boolean pidActiveR = false;   //for rotate

// Extra Information
const float wheel_radius = 3.35;  // in cm
const float circumference = 2 * M_PI * wheel_radius;
const float tickDistance = (float)circumference/1500.0;

// For incoming Serial String
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

unsigned long previousMillis = 0;
const long interval = 10;

float totalDistance = 0;
double travelDist = 0;
boolean doneRotate = false;
boolean doneStraight = false;

struct motorPWM{
  double pwm1;
  double pwm2;
  double rest;
};
motorPWM rotateRight;
motorPWM rotateLeft;

const int total = 50;
int errorArray[total];
int readIndex = 0;
 
void setup() {
  Serial.begin(115200);
  inputString.reserve(100);
  
  encL.write(0);
  encR.write(0);
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);

  initEEPROM();
  
  // initalizing PID
  pidL.SetMode(MANUAL);       // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidRotate.SetMode(AUTOMATIC);
  pidL.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
  pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
  pidRotate.SetTunings(rotatePID.kp, rotatePID.ki, rotatePID.kd);
  pidL.SetSampleTime(interval);      // Sample time for PID
  pidR.SetSampleTime(interval);
  pidRotate.SetSampleTime(interval);
  pidL.SetOutputLimits(0,255);  // min/max PWM
  pidR.SetOutputLimits(0,255);
  pidRotate.SetOutputLimits(0,255);
}

void loop() {

  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    long encCurr1 = encL.read();
    long encCurr2 = encR.read();
    encoderL += encCurr1;
    encoderR += encCurr2;
    resetEncoders();

    float distance1 = (float)encCurr1*tickDistance;
    spd1 = (float)distance1*(1000.0/interval);
    float distance2 = (float)encCurr2*tickDistance;
    spd2 = (float)distance2*(1000.0/interval);

    totalDistance += (distance1 + distance2)/2;    //total Distance
  }

  pidL.Compute();
  pidR.Compute();
  pidRotate.Compute();

  if(pidActiveS)  moveStraight(travelDist);
  else if (pidActiveR){
    updateErrorArray();
    correctHeading();
  }
  else{
    motorL.setDir(BRAKE);
    motorR.setDir(BRAKE);
    motorL.setPWM(0);
    motorR.setPWM(0);
  }
}

void(* resetFunc) (void) = 0;//declare reset function at address 0

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;
    
    switch(inputString[0]){
      case 'V':
        // COMMAND:  V,speed_motor_left,speed_motor_right,steps\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        val2 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        travelDist = inputString.substring(c1).toFloat();
        
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        vel1 = val1;
        vel2 = val2;
        if(DEBUG){
          Serial.print("Velocity 1 ");
          Serial.println(vel1);
          Serial.print("Velocity 2 ");
          Serial.println(vel2);
        }         
        
        if(vel1>0)
          pidL.SetMode(AUTOMATIC);
        else
          pidL.SetMode(MANUAL);
        if(vel2>0)
          pidR.SetMode(AUTOMATIC);
        else
          pidR.SetMode(MANUAL);

        pidRotate.SetMode(AUTOMATIC);
        pidActiveS = true;
        pidActiveR = false;
        totalDistance = 0;
        break;
        
      case 'P':
        // COMMAND:  P,kp,ki,kd,1/2\n
        float p,i,d;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        p = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        i = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        d = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val1 = inputString.substring(c1).toInt();
        
        if(val1==1) {
          motorPIDL.kp = p;
          motorPIDL.ki = i;
          motorPIDL.kd = d;
          pidL.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
          EEPROM.put((const int)MAGICADDRESS, motorPIDL);
        }
        else if(val1==2){
          motorPIDR.kp = p;
          motorPIDR.ki = i;
          motorPIDR.kd = d;
          pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
        }
        else if (val1==3){
          rotatePID.kp = p;
          rotatePID.ki = i;
          rotatePID.kd = d;
          pidRotate.SetTunings(rotatePID.kp, rotatePID.ki, rotatePID.kd);
        }
        if (DEBUG){
        Serial.print("kp: ");
        Serial.println(p);
        Serial.print("ki: ");
        Serial.println(i);
        Serial.print("kd: ");
        Serial.println(d);
        Serial.println('h');
        }
        break;
      
      case 'L':
        // COMMAND:  L,pwm_motor_left, pwm_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        val2 = inputString.substring(c1).toInt();
        
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        
        pidActiveS= false;
        pidActiveR= false;
        pidL.SetMode(MANUAL);
        pidR.SetMode(MANUAL);
        pidRotate.SetMode(MANUAL);
        pwm1 = val1;
        pwm2 = val2;
        motorL.setPWM(pwm1);
        motorR.setPWM(pwm2);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        break;
      
      case 'G':
        pidRotate.SetMode(AUTOMATIC);
        if(inputString[2]  == 'R'){
          inputString = inputString.substring(2);
          c1 = inputString.indexOf(',')+1;
          c2 = inputString.indexOf(',',c1);
          int pwmL = inputString.substring(c1,c2).toInt();
          rotateRight.pwm1 = pwmL;
          
          c1 = c2+1;
          c2 = inputString.indexOf(',',c1);
          int pwmR = inputString.substring(c1,c2).toInt();
          rotateRight.pwm2 = pwmR;
          
          c1 = c2+1;
          val1 = inputString.substring(c1).toInt(); 
          rotateRight.rest = val1;
          
          motorL.setDir(FORWARD);
          motorR.setDir(BACKWARD);
          rotatePWM(pwmL,pwmR,val1);
        }
        else if (inputString[2]  == 'L'){
          inputString = inputString.substring(2);
          c1 = inputString.indexOf(',')+1;
          c2 = inputString.indexOf(',',c1);
          int pwmL = inputString.substring(c1,c2).toInt();
          rotateLeft.pwm1 = pwmL;
          
          c1 = c2+1;
          c2 = inputString.indexOf(',',c1);
          int pwmR = inputString.substring(c1,c2).toInt();
          rotateLeft.pwm2 = pwmR;
          
          c1 = c2+1;
          val1 = inputString.substring(c1).toInt(); 
          rotateLeft.rest = val1;
          
          motorL.setDir(BACKWARD);
          motorR.setDir(FORWARD);
          rotatePWM(pwmL,pwmR,val1);
        }
        pidActiveR = true;
      break;

      case 'H':
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        currentH = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        setpoint = inputString.substring(c1).toFloat();
      break;
      
      case 'R':
        // COMMAND:  R\n
        Serial.print("r,");
        Serial.print(encoderL);
        Serial.print(',');
        Serial.print(encoderR);
        Serial.println();
        break;
      
      case 'X':
        // COMMAND: X\n
        resetEncoders();
        encoderL = 0;
        encoderR = 0;
        Serial.println('i');
        break;
      
      //for path plan
      case 'C':
        //COMMAND: C,L1/R1\n   OR   C,S\n
        pidRotate.SetMode(AUTOMATIC);
        if(inputString[2] == 'L'){
          motorL.setDir(BACKWARD);
          motorR.setDir(FORWARD);
          
          if(inputString[3] == '1'){
            rotatePWM(rotateLeft.pwm1,rotateLeft.pwm1,rotateLeft.rest);  
          }
          else if(inputString[3] == '2'){
            rotatePWM(rotateLeft.pwm1,rotateLeft.pwm1,rotateLeft.rest * 2);  
          }
        }
        else if(inputString[2] == 'R'){
          motorL.setDir(FORWARD);
          motorR.setDir(BACKWARD);
          
          if(inputString[3] == '1'){
            rotatePWM(rotateRight.pwm1,rotateRight.pwm1,rotateRight.rest);  
          }
          else if(inputString[3] == '2'){
            rotatePWM(rotateRight.pwm1,rotateRight.pwm1,rotateRight.rest * 2);  
          }
        }
        else if(inputString[2] == 'S'){
          pidL.SetMode(AUTOMATIC);
          pidR.SetMode(AUTOMATIC);
          pidActiveS = true;
          pidActiveR = false;
          totalDistance = 0;
          resetEncoders(); 
        }
      break;
      
      default:
        Serial.print("UNKNOWN COMMAND: ");
        Serial.println(inputString);
        break;
    }
}

void resetEncoders(void){
  encL.write(0);
  encR.write(0);
}

void initEEPROM(void){
  int checkEEPROM=0;
  EEPROM.get(0, checkEEPROM);
  if(checkEEPROM==MAGICADDRESS){
    if(DEBUG) Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorPIDL);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }
  else{
    //set default values
    if(DEBUG) Serial.println("Setting Default Values.");
    EEPROM.put(0, MAGICADDRESS);
    motorPIDL.kp = 1.0;
    motorPIDL.ki = 0.0;
    motorPIDL.kd = 0.0;
    EEPROM.put((const int)MAGICADDRESS, motorPIDL);

    motorPIDR.kp = 1.0;
    motorPIDR.ki = 0.0;
    motorPIDR.kd = 0.0;
    EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }  
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void moveStraight(float steps){
  if(totalDistance < steps){
    if(vel1>0) motorL.setPWM(pwm1);
    else motorL.setPWM(0);
    if(vel2>0) motorR.setPWM(pwm2);
    else motorR.setPWM(0);
  }
  else{
    motorL.setDir(BRAKE);
    motorR.setDir(BRAKE);
    motorL.setPWM(0);
    motorR.setPWM(0);
    totalDistance = 0;
    pidActiveS = false;
    pidL.SetMode(MANUAL);
    pidR.SetMode(MANUAL);
    Serial.println("START");    //send this to mega to start sending Megneto readings 
    pidActiveR = true;
    doneStraight = true;
  }
}

void correctHeading(){
 /*   Serial.print(currentH);
   Serial.print(",");
   Serial.print(setpoint);
   Serial.print(",");
   Serial.print(pidRotate.GetError());
   Serial.print(",");
   Serial.println(pwmRotate);
   */
   if(pidRotate.GetError()<=(-3)){
        pidRotate.SetControllerDirection(REVERSE);
        motorL.setDir(FORWARD);
        motorR.setDir(BACKWARD);
        motorL.setPWM(pwmRotate);
        motorR.setPWM(pwmRotate);
      }
    else if (pidRotate.GetError() >= 3){
        pidRotate.SetControllerDirection(DIRECT);
        motorL.setDir(BACKWARD);
        motorR.setDir(FORWARD);
        motorL.setPWM(pwmRotate);
        motorR.setPWM(pwmRotate);
      }
    else if ((pidRotate.GetError() < 3) && (pidRotate.GetError() > (-3)) && checkErrorArray){
        motorL.setDir(BRAKE);
        motorR.setDir(BRAKE);
        motorL.setPWM(0);
        motorR.setPWM(0);
        pidRotate.SetMode(MANUAL);
        //pidActiveS = true;
        Serial.println("STOP");    //send this to mega to stop sending Megneto readings 
        delay(10);
        
        if(doneStraight) {
          Serial.println("STRAIGHT");
          doneStraight = false;
        }
        else if (!doneStraight) Serial.println("ROTATE");
        
        pidActiveR = false;
        resetEncoders();    //reset encoders
      }
    else{
        motorL.setDir(BRAKE);
        motorR.setDir(BRAKE);
        motorL.setPWM(0);
        motorR.setPWM(0);
      }
}


void rotatePWM(int pwm1, int pwm2, int val){

  motorL.setPWM(pwm1);
  motorR.setPWM(pwm2);
  delay(val);

  motorL.setDir(BRAKE);
  motorR.setDir(BRAKE);
  motorL.setPWM(0);
  motorR.setPWM(0);

  pidActiveR = true;
  pidActiveS = false;
  Serial.println("START");
}

void initErrorArray(){
  for(int i ; i<total; i++){
    errorArray[i]=9; 
  }
}

void updateErrorArray(){
  errorArray[readIndex] = pidRotate.GetError();
  readIndex++;
  if(readIndex >= total) readIndex = 0;
}

bool checkErrorArray(){
  int count;
  for(int i ; i<total; i++){
    if((errorArray[i]==-2)||(errorArray[i]==-1)||(errorArray[i]==0)||(errorArray[i]==1)||(errorArray[i]==2))
    {
      count++; 
    }
  }

  if (count >= total) return true;
  else return false;
}


