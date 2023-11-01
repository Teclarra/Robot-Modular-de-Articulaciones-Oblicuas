

#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

//===================================== TCA9548 Multiplexer i2C
#define tcaAddress 0x70
enum muxUsersIndex {magEnc1, magEnc2, muxFree2, muxFree3, muxFree4, muxFree5, muxFree6, muxFree7};

//===================================== AS5600 magnetic encoder
#include <AS5600.h>
AS5600 magnEncoder;
const float magnEncStepsDegrees = 4096/360;
Servo myservo1, myservo2;
int pos=180;
int pos1=180, pos2=180;
double kp = 3 , ki = 0.5 , kd = 0.005;             // modify for optimal performance
double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
PID myPID1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT);
PID myPID2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);
int flag=0;
unsigned long tiempo_cp=0;

void setup()
{
    Wire.begin();
    Serial.begin(115200);             // Define baud rate
    myservo1.attach(7);
    myservo2.attach(6);
    setpoint1=pos;
    checkpos1(pos, magEnc1);
    checkpos2(pos, magEnc2);
    tiempo_cp=millis();
    
  //TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID1.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID1.SetSampleTime(1);  // refresh rate of PID controller
  myPID1.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  myPID2.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID2.SetSampleTime(1);  // refresh rate of PID controller
  myPID2.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
    delay(1500);
}

void loop()
{

/*Serial.print (pos1);
Serial.print("           ");
Serial.print (input1);
Serial.print("           ");
Serial.print (output1);
Serial.print("           ");
Serial.print (pos2);
Serial.print("           ");
Serial.print (input2);
Serial.print("           ");
Serial.println (output2);*/

checkpos1(pos1, magEnc1);
checkpos2(pos2, magEnc2);

/*if(millis()-tiempo_cp>5000){
  if (flag==1) {
     pos1=90;
     pos2=270;
     flag=0;}
  else if(flag==0){
      pos1=270;
      pos2=90;
      flag=1;}  
  tiempo_cp = millis();}*/
 
}


void checkpos1(int p, int magEncIndex){

   tcaselect(magEncIndex);
   setpoint1=p;
   input1 = (magnEncGetPosDegrees(magEncIndex));
   myPID1.Compute();                  
   myservo1.write(map (output1, -90, 90, 180, 0));
   //delay(50);
  }
void checkpos2(int p, int magEncIndex){

   tcaselect(magEncIndex);
   setpoint2=p;
   input2 = (magnEncGetPosDegrees(magEncIndex));
   myPID2.Compute();                  
   myservo2.write(map (output2, -90, 90, 180, 0));
   //delay(50);

  }

//===================================== Mux i2C channel selection
void tcaselect(uint8_t i){
     
    if (i > 7) return;
    Wire.beginTransmission(tcaAddress);
    Wire.write(1 << i);
    Wire.endTransmission();
    delay(10);
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
