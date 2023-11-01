

#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

//===================================== TCA9548 Multiplexer i2C
#define tcaAddress 0x70
enum muxUsersIndex {magEnc, magEnc2, magEnc3, magEnc4, magEnc5, magEnc6, muxFree6, muxFree7};

//===================================== AS5600 magnetic encoder
#include <AS5600.h>
AS5600 magnEncoder;
const float magnEncStepsDegrees = 4096/360;
Servo myservo;
int pos=250;
double kp = 2 , ki = 0.5 , kd = 0.005;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
int flag=0;
unsigned long tiempo_cp=0;

void setup()
{
    Wire.begin();
    Serial.begin(115200);             // Define baud rate
    myservo.attach(7);
    setpoint=pos;
    tiempo_cp=millis();
    
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
 
    delay(1500);
}

void loop()
{

Serial.print (pos);
Serial.print("           ");
Serial.print (magnEncGetPosDegrees(magEnc6));
Serial.print("           ");
Serial.println (output);

checkpos(myservo,pos,magEnc6);

/*if(millis()-tiempo_cp>5000){
  if (flag==1) {
     pos=90;
     flag=0;}
  else if(flag==0){
      pos=270;
      flag=1;}  
  tiempo_cp = millis();}*/
 
}


void checkpos(Servo servo, int p, uint8_t magEncIndex){

   tcaselect(magEncIndex);
   setpoint=p;
   input = (magnEncGetPosDegrees(magEnc));
   myPID.Compute();                  
   servo.write(map (output, -90, 90, 180, 0));
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
int magnEncGetPosition(int magnEncIndex)
{
    tcaselect(magnEncIndex);
    return magnEncoder.readAngle();
}

float magnEncGetPosDegrees(int magnEncIndex)
{
    tcaselect(magnEncIndex);
    return magnEncoder.readAngle()/magnEncStepsDegrees;
}
