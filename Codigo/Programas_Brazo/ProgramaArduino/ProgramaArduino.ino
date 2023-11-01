//===================================== Required libraries
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

//===================================== TCA9548 Multiplexer i2C
#define tcaAddress 0x70
enum muxUsersIndex {magEnc1, magEnc2, magEnc3, magEnc4, magEnc5, magEnc6, muxFree1, muxFree2};

//===================================== AS5600 magnetic encoder
#include <AS5600.h>
AS5600 magnEncoder;
const float magnEncStepsDegrees = 4096/360;

//===================================== Servo and PID
Servo myservo1, myservo2, myservo3, myservo4, myservo5, myservo6;
float pos0=180;                                                     //initial position
float pos1=180, pos2=180, pos3=180, pos4=180, pos5=180, pos6=180;

double kp = 2 , ki = 0.5 , kd = 0.005;                              // modify for optimal performance

double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
double input3 = 0, output3 = 0, setpoint3 = 0;
double input4 = 0, output4 = 0, setpoint4 = 0;
double input5 = 0, output5 = 0, setpoint5 = 0;
double input6 = 0, output6 = 0, setpoint6 = 0;

PID myPID1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT);
PID myPID2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);
PID myPID3(&input3, &output3, &setpoint3, kp, ki, kd, DIRECT);
PID myPID4(&input4, &output4, &setpoint4, kp, ki, kd, DIRECT);
PID myPID5(&input5, &output5, &setpoint5, kp, ki, kd, DIRECT);
PID myPID6(&input6, &output6, &setpoint6, 0.5, ki, kd, DIRECT);

//===================================== Serial communication
String StrQ1, StrQ2, StrQ3, StrQ4, StrQ5, StrQ6;

void setup()
{
    Wire.begin();
    Serial.begin(115200);             // Define baud rate
    
    myservo1.attach(2);
    myservo2.attach(3);
    myservo3.attach(4);
    myservo4.attach(5);
    myservo5.attach(6);
    myservo6.attach(7);
    
    checkpos1(pos0, magEnc1);         //initial possition
    checkpos2(pos0, magEnc2);
    checkpos1(pos0, magEnc1);
    checkpos2(pos0, magEnc2);
    checkpos1(pos0, magEnc1);
    checkpos2(pos0, magEnc2);
    
    
    myPID1.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID1.SetSampleTime(1);  // refresh rate of PID controller
    myPID1.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
    
    myPID2.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID2.SetSampleTime(1);  // refresh rate of PID controller
    myPID2.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

    myPID3.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID3.SetSampleTime(1);  // refresh rate of PID controller
    myPID3.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

    myPID4.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID4.SetSampleTime(1);  // refresh rate of PID controller
    myPID4.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
        
    myPID5.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID5.SetSampleTime(1);  // refresh rate of PID controller
    myPID5.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

    myPID6.SetMode(AUTOMATIC);   //set PID in Auto mode
    myPID6.SetSampleTime(1);  // refresh rate of PID controller
    myPID6.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
    
    delay(1500);
}

void loop()
{
  //Lectura de datos por SERIAL
  if (Serial.available()) 
  {
    StrQ1 = Serial.readStringUntil(',');
    StrQ2 = Serial.readStringUntil(',');
    StrQ3 = Serial.readStringUntil(',');
    StrQ4 = Serial.readStringUntil(',');
    StrQ5 = Serial.readStringUntil(',');
    StrQ6 = Serial.readStringUntil(',');
    
    pos1 = StrQ1.toInt()+180;
    pos2 = StrQ2.toInt()+180;
    pos3 = StrQ3.toInt()+180;
    pos4 = StrQ4.toInt()+180;
    pos5 = StrQ5.toInt()+180;
    pos6 = StrQ6.toInt()+180;
  } 

  checkpos1(pos1, magEnc1);
  checkpos2(pos2, magEnc2);
  checkpos3(pos3, magEnc3);
  checkpos4(pos4, magEnc4);
  checkpos5(pos5, magEnc5);
  checkpos6(pos6, magEnc6);

}

//===================================== Closed loop position control

void checkpos1(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint1=p;
   input1 = (magnEncGetPosDegrees(magEncIndex));
   myPID1.Compute();                  
   myservo1.write(map (output1, -90, 90, 180, 0));
   //delay(50);
  }
  
void checkpos2(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint2=p;
   input2 = (magnEncGetPosDegrees(magEncIndex));
   myPID2.Compute();                  
   myservo2.write(map (output2, -90, 90, 180, 0));
   //delay(50);
 }
 
 void checkpos3(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint3=p;
   input3 = (magnEncGetPosDegrees(magEncIndex));
   myPID3.Compute();                  
   myservo3.write(map (output3, -90, 90, 180, 0));
   //delay(50);
 }

 void checkpos4(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint4=p;
   input4 = (magnEncGetPosDegrees(magEncIndex));
   myPID4.Compute();                  
   myservo4.write(map (output4, -90, 90, 180, 0));
   //delay(50);
 }

 void checkpos5(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint5=p;
   input5 = (magnEncGetPosDegrees(magEncIndex));
   myPID5.Compute();                  
   myservo5.write(map (output5, -90, 90, 180, 0));
   //delay(50);
 }
 
 void checkpos6(int p, int magEncIndex)
{
   tcaselect(magEncIndex);
   setpoint6=p;
   input6 = (magnEncGetPosDegrees(magEncIndex));
   myPID6.Compute();                  
   myservo6.write(map (output6, -90, 90, 180, 0));
   //delay(50);
 }

//===================================== Mux i2C channel selection
void tcaselect(uint8_t i)
{  
    if (i > 7) return;
    Wire.beginTransmission(tcaAddress);
    Wire.write(1 << i);
    Wire.endTransmission();
    //delay(1);
}

//===================================== Magnetic encoder
int magnEncGetPosition(int magEncIndex)
{
    tcaselect(magEncIndex);
    return magnEncoder.readAngle();
}

float magnEncGetPosDegrees(int magnEncIndex)
{
    tcaselect(magnEncIndex);
    return magnEncoder.readAngle()/magnEncStepsDegrees;
}
