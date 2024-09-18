#include <Arduino.h>
#include <BH1750FVI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <OneWire.h>

//uart codes for sensors 
#define DHT_TEMP_SENSOR_CODE "0001 "
#define DHT_HUMIDITY_SENSOR_CODE "0002 "
#define MQ135_SENSOR_CODE "0003 "
#define LIGHT_INTENSITY_SENSOR_CODE "0004 "
#define UV_SENSOR_CODE "0005 "
#define WATER_TEMP_SENSOR_CODE "0006 "
#define WATER_HEIGHT_SENSOR_CODE "0007 "
#define GRID_VOLTAGE_SENSOR_CODE "0008 "
#define NEIGHBORHOOD_GENERATOR_VOLTAGE_SENSOR_CODE "0009 "


#define DHTTYPE DHT22
#define DHTPIN 4 
#define BH17ADDR_PIN 7
#define ULTRASONIC_ECHOPIN 11
#define ULTRASONIC_TRIGPIN 10
#define WATERPROOF_TEMP 8
#define AC_LINE_SWAP_INVERTED_RELAY 12
#define GYML8511_UV_EN_PIN 5
#define GYML8511_UV_READ_PIN A3
#define MQ135PIN A0 
#define VOLTAGE_SENSOR_ZMPT A2
//I2C SCL A5
//I2C SDA A4

DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;
OneWire ds(WATERPROOF_TEMP);
BH1750FVI bh17LightLux(BH17ADDR_PIN);

void DHTMeasureHumidityAndSend();
void DHTMeasureTempAndSend();
void MQ135MeasureAirQualityAndSend();
void ultrasonicMeasureDistance();
void waterTemperature();
void lightIntensity();
void UVLightGy();
double measureVoltage();
int averageAnalogRead(int pinToRead);
void mainsAndGeneratorVoltage();

void outputValuesFloat(char *sensorCode, float value);
void outputValuesInt(char *sensorCode, uint16_t value);
void setRelayState(boolean state);

boolean relayState = false;
/*
 * Setup function. Here we do the basics
 */
void setup(void)
{
  Serial.begin(9600); 
  pinMode(ULTRASONIC_ECHOPIN, INPUT);
  pinMode(ULTRASONIC_TRIGPIN, OUTPUT);
  pinMode(WATERPROOF_TEMP, INPUT_PULLUP);
  pinMode(GYML8511_UV_READ_PIN, INPUT);
  pinMode(GYML8511_UV_EN_PIN, INPUT);
  pinMode(AC_LINE_SWAP_INVERTED_RELAY, OUTPUT);
  // Initialize device.
  dht.begin();
  ds.begin(WATERPROOF_TEMP);
  bh17LightLux.begin();



}


/*
 * Main function. It will request the tempC from the sensors and display on Serial.
 */
void loop(void)
{
  setRelayState(HIGH);
  delay(100);
  mainsAndGeneratorVoltage();
  
  //mainsAndGeneratorVoltage();
  //waterTemperature();
  //ultrasonicMeasureDistance();
  //MQ135MeasureAirQualityAndSend();
  //DHTMeasureTempAndSend();
  //DHTMeasureHumidityAndSend();
  delay(500);
}

void UVLightGy(){
  digitalWrite(GYML8511_UV_EN_PIN, HIGH);
  delay(10);
  int val = averageAnalogRead(GYML8511_UV_READ_PIN);
  digitalWrite(GYML8511_UV_EN_PIN, LOW);
  float val5Volt = (float)val*5/1023;
  
  float valFinal = (val5Volt - 1.115)*15/(3 - 1); // value in mW/cm2
  
  if (valFinal <= 0) valFinal = 0;
  outputValuesFloat(UV_SENSOR_CODE, valFinal);
};

void lightIntensity(){
  float lux;
  //do dynamic remeasuring for different situations
  float midRes = bh17LightLux.getLightIntensityHighRes();
  if (midRes >= 60000) {
    lux = bh17LightLux.getLightIntensityLowRes();
  } else if (midRes <= 10) {
    lux = bh17LightLux.getLightIntensityHighRes2();
  } else lux = midRes;
  
  outputValuesFloat(LIGHT_INTENSITY_SENSOR_CODE, lux);
};

void mainsAndGeneratorVoltage(){
  setRelayState(HIGH);
  delay(1000);
  double volt = measureVoltage();
  if (volt > 15)
  {
    outputValuesFloat(GRID_VOLTAGE_SENSOR_CODE, volt);
  } else
  {
    setRelayState(LOW);
    delay(1000);
    volt = measureVoltage();
    outputValuesFloat(NEIGHBORHOOD_GENERATOR_VOLTAGE_SENSOR_CODE, volt);
  }
  
  
};
double measureVoltage(){
  int FREQUENCY =  50;
  int ZERO_VAC =  508;

  uint32_t period = 1000000 / FREQUENCY;
  uint32_t Vsum = 0;
  uint32_t measurements_count = 0;
  int16_t Vnow;
  uint32_t t_start = micros();
  while (micros() - t_start < period) {
      int16_t raw = analogRead(VOLTAGE_SENSOR_ZMPT);
      Vnow = raw - ZERO_VAC;
      Vsum += Vnow*Vnow;
      measurements_count++;
  }
  double Vrms = sqrt(Vsum / measurements_count);
  
  return Vrms; //requires multiplication on the esp side. to convert to ac voltage value
};

void waterTemperature(){
  byte i;
  byte type_s = 0; //  Chip = DS18B20
  byte data[9];
  byte addr[8];
  float celsius;


  if ( !ds.search(addr)) {
    Serial.println("water temperature: No addresses found.");
    ds.reset_search();
    delay(250);
    return;
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("water temperature: CRC is not valid!");
    return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  
  celsius = (float)raw / 16.0;
  outputValuesFloat(WATER_TEMP_SENSOR_CODE, celsius);
};

void ultrasonicMeasureDistance(){
  float duration, distance;
  // Set the trigger pin LOW for 2uS
  digitalWrite(ULTRASONIC_TRIGPIN, LOW);
  delayMicroseconds(2);

  // Set the trigger pin HIGH for 20us to send pulse
  digitalWrite(ULTRASONIC_TRIGPIN, HIGH);
  delayMicroseconds(20); //20

  // Return the trigger pin to LOW
  digitalWrite(ULTRASONIC_TRIGPIN, LOW);

  // Measure the width of the incoming pulse
  duration = pulseIn(ULTRASONIC_ECHOPIN, HIGH);

  distance = (duration / 2) * 0.343;
  distance = distance / 10; // cm
  outputValuesFloat(WATER_HEIGHT_SENSOR_CODE, distance);
  
};

void MQ135MeasureAirQualityAndSend(){

  float sensorValueRaw = averageAnalogRead(MQ135PIN);
  float sensorValue = (float)sensorValueRaw/1023;
  outputValuesFloat(MQ135_SENSOR_CODE, sensorValue);
};

void DHTMeasureHumidityAndSend(){
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    outputValuesFloat(DHT_HUMIDITY_SENSOR_CODE, event.relative_humidity);
  }
};
void DHTMeasureTempAndSend(){
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    outputValuesFloat(DHT_TEMP_SENSOR_CODE, event.temperature);
  }
};

int averageAnalogRead(int pinToRead)
{
  uint8_t numberOfReadings = 8;
  uint16_t runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  //out of loop
  runningValue /= numberOfReadings;

  return(runningValue);  
};

void outputValuesFloat(char *sensorCode, float value){
  String code = sensorCode;
  code += value;
  Serial.println(code);
}

void outputValuesInt(char *sensorCode, uint16_t value){
  String code = sensorCode;
  code += value;
  Serial.println(code);
}

void setRelayState(boolean state) {
  if (relayState != state) {
    relayState = state;
    digitalWrite(AC_LINE_SWAP_INVERTED_RELAY, relayState);
  }
}