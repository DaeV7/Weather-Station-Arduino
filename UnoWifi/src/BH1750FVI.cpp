#include <Wire.h>
#include <Arduino.h>

#include <BH1750FVI.h>

void BH1750FVI::begin(){
  Wire.begin();
  pinMode(addressPin, OUTPUT);
  setAddressL();
  I2CWrite(I2C_deviceStatePowerUp);
  I2CWrite(I2C_deviceStateReset);
  I2CWrite(currentMode);
  delay(120);
};

void BH1750FVI::setMode(eDeviceMode_t mode){
  currentMode = mode;
  delay(10);
  I2CWrite(mode);
};

uint16_t BH1750FVI::getLightIntensity(eDeviceMode_t mode){
  I2CWrite(I2C_deviceStateReset);
  delay(10);
  I2CWrite(mode);  
  delay(160);

  I2CWrite(mode);
  delay(120);
  uint16_t val = 0;
  Wire.requestFrom(currentDeviceAddress, 2);
  val = Wire.read();
  val <<= 8; //shift 8 bits
  val |= Wire.read(); // or operation, get the other 8 bits 

  return val;
};

float BH1750FVI::getLightIntensityHighRes(){
  
  
  return getLightIntensity(I2c_DevModeContHighRes)/1.2;
};

float BH1750FVI::getLightIntensityHighRes2(){
  
  return getLightIntensity(I2c_DevModeContHighRes2)/2.4;
};

float BH1750FVI::getLightIntensityLowRes(){
  
  return getLightIntensity(I2c_DevModeContLowRes)/1.146;
};

void BH1750FVI::I2CWrite(uint8_t Data){
  Wire.beginTransmission(currentDeviceAddress);
  Wire.write(Data);
  Wire.endTransmission();
};

void BH1750FVI::setAddressL(){
  digitalWrite(addressPin, LOW);
  currentDeviceAddress = deviceAddress_L;
};

void BH1750FVI::setAddressH(){
  digitalWrite(addressPin, HIGH);
  currentDeviceAddress = deviceAddress_H;
};
