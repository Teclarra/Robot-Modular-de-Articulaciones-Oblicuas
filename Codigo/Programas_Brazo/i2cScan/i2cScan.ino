#include <Wire.h>

uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
        if (!wait) return 4;
        Wire.beginTransmission(address);
        while (length) {
                Wire.write(*data++);
                length--;
        }
        return Wire.endTransmission(sendStop);
}
void scanI2CBus(byte from_addr, byte to_addr, void(*callback)(byte address, byte result) ) 
{
  byte rc;
  byte data = 0;
  for( byte addr = from_addr; addr <= to_addr; addr++ ) {
    rc = twi_writeTo(addr, &data, 0, 1, 0);
    callback( addr, rc );
  }
}

void scanFunc( byte addr, byte result ) {
  Serial.print("addr: ");
  Serial.print(addr,DEC);
  Serial.print( (result==0) ? " Encontrado!":"       ");
  Serial.print( (addr%4) ? "\t":"\n");
}

const byte start_address = 8;
const byte end_address = 119;

void setup()
{
    Wire.begin();

    Serial.begin(9600);
    Serial.print("Escaneando bus I2C...");
    scanI2CBus( start_address, end_address, scanFunc );
    Serial.println("\nTerminado");
}

void loop() 
{
    delay(1000);
}
