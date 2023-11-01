
#include <Wire.h>
#include <Servo.h>

//===================================== TCA9548 Multiplexer i2C
#define tcaAddress 0x70
enum muxUsersIndex {magEnc1, magEnc2, muxFree2, muxFree3, muxFree4, muxFree5, muxFree6, muxFree7};

//===================================== AS5600 magnetic encoder
#include <AS5600.h>
AS5600 magnEncoder;
const float magnEncStepsDegrees = 4096/360;
Servo myservo;

void setup()
{
    Wire.begin();
    Serial.begin(115200);             // Define baud rate
    myservo.attach(7);
    //myservo.write(160);
 
    delay(1500);
}

void loop()
{

Serial.print (magnEncGetPosDegrees(magEnc1));
Serial.println (magnEncGetPosDegrees(magEnc2));
   
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
