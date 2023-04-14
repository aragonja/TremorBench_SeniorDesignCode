#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include "arduinoFFT.h"

int buttonPin = D0; 
int output;

arduinoFFT FFT;
using namespace std;

const uint8_t MPU6050SlaveAddress = 0x68;     // MPU6050 Slave Device Address // I2C communication
const uint8_t scl = D6;                       // Select SDA and SCL pins for I2C communication
const uint8_t sda = D7;
const uint16_t AccelScaleFactor = 8192;      // Sensitivity scale factor respective to full  +2g

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

//declaring variables global
int16_t AccelX, AccelY, AccelZ;               
//configure read function
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure write function
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2 max of 3000 samples =  3 seconds
const double samplingFrequency = 1000;

//input vectors receive computed results from FFT
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00       // Serial Clock Index = 0 
#define SCL_TIME 0x02        // Serial Clock Time = 1 second
#define SCL_FREQUENCY 0x025  // Serial Clock  = 37Hz
#define SCL_PLOT 0x03        // Serial Clock Plot = 3

void PrintVector(double *vData, uint8_t scaleType) //refrence from example code
{
  for (uint16_t i = 0; i < 256; i++)
  {
    double abs;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_TIME:
        abs = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abs = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    //Serial.print(abs);
    //if(scaleType==SCL_FREQUENCY)
    //  Serial.print("Hz");
    //Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void setup()
{
  Serial.begin(9600);   //need at least 8000 baud 
  Wire.begin(sda, scl);
  Serial.print("Waiting: ");
  MPU6050_Init();
  //pinMode(buttonPin, OUTPUT);
}

void loop()
{
  double Az;
  //Button configuration
  //output = digitalRead(buttonPin);
  //if(output == LOW){
  //  output = digitalRead(buttonPin);
  //}
  //else{
    for (uint16_t i = 0; i < samples; i++)
    {
      //Input accelerometer data readings
      Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
      Az = float(AccelX/AccelScaleFactor);
      vReal[i] = Az;              //Accelerometer Data 
      vImag[i] = 0.0;             //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
      delay(1);
      //Serial.println(Az);
    }
    FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(FFT_FORWARD);                         /* Compute FFT */
    FFT.ComplexToMagnitude();                         /* Compute magnitudes */
    PrintVector(vReal, SCL_FREQUENCY);
    while(1){
      delay(100000);   //forever loop
      //output = digitalRead(buttonPin);
      //yield();
    }  
  //}
}  