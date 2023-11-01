
#include <Wire.h> //This is for i2C


//===================================== TCA9548 Multiplexer i2C
#define tcaAddress 0x70
enum muxUsersIndex {magEnc, muxFree1, muxFree2, muxFree3, muxFree4, muxFree5, muxFree6, muxFree7};

//===================================== AS5600 magnetic encoder
#include <AS5600.h>
AS5600 magnEncoder;
const float magnEncStepsDegrees = 4096/360;



//---------------------------------------------------------------------------
//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)


void setup()
{
 
  Serial.begin(115200); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(); //start i2C
  Wire.setClock(800000L); //fast clock

  checkMagnetPresence(magEnc); //check the magnet (blocks until magnet is found)
}

void loop()
{
  
}


void checkMagnetPresence(int magnEncIndex)
{
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while ((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    tcaselect(magnEncIndex);
    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while (Wire.available() == 0); //wait until it becomes available
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)
  }

  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet - 100111 - DEC: 39
  //ML: Too weak magnet - 10111 - DEC: 23
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  //delay(1000);
}

void tcaselect(uint8_t i){
     
    if (i > 7) return;
    Wire.beginTransmission(tcaAddress);
    Wire.write(1 << i);
    Wire.endTransmission();
    delay(10);
}

//===================================== Magnetic encoder
float magnEncGetPosition(int magnEncIndex)
{
    tcaselect(magnEncIndex);
    return magnEncoder.readAngle();
}

float magnEncGetPosDegrees(int magnEncIndex)
{
    tcaselect(magnEncIndex);
    return magnEncoder.readAngle()/magnEncStepsDegrees;
}
