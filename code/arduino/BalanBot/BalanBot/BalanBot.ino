
#include <EEPROM.h>
#include <Wire.h>
#include "Kalman.h" 
#include "I2C.h"
#include "PinChangeInt.h"

/* Motor */
#define PWM_L 6  //M1
#define PWM_R 5  //M2
#define DIR_R1 8
#define DIR_R2 7
#define DIR_L1 3
#define DIR_L2 4

/* Encoder */
#define SPD_INT_R 11   //interrupt 
#define SPD_PUL_R 12   
#define SPD_INT_L 10   //interrupt 
#define SPD_PUL_L 9

#define BUZZER 13
#define LED 13

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculated using the gyro only
double kalAngleX, kalAngleY; // Angle calculate using a Kalman filter

uint32_t timer;
uint32_t LEDTimer;

uint8_t i2cData[14]; // Buffer for I2C data

int pwm_l, pwm_r;

int encCountLeft, encCountRight;  // encoder coutns
double Position_Car;              // encoder counts
double Angle_Car;                 // degrees
double Gyro_Car;                  // degrees per second
double Speed_Car;                 // encoder counts per second
double Speed_Command;             // commanded speed

double Ka,Kg,Ki,Ks,Kp;
double angle_zero;

bool blinkState = false;

Kalman kalmanX;
Kalman kalmanY;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(BUZZER, OUTPUT);
  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT); 
  
  ///init variables
  encCountLeft = 0;
  encCountRight = 0;
  Position_Car = 0;
  Speed_Command = 0;
  
  Ka = 50.0;  //originally 25
  Kg = 3.5;   //originally 3.5
  Ki = 0.1;
  Ks = 0.0;    
  Kp = 0.0;  
  angle_zero = -2.0; 

  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(200); // Wait for sensor to stabilize

  /* Set kalman and gyro initial angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
 
  PCintPort::attachInterrupt(SPD_INT_L, Encoder_L,FALLING);
  PCintPort::attachInterrupt(SPD_INT_R, Encoder_R,FALLING);
  
  timer = micros();
  LEDTimer = millis();  
}

void loop() 
{
  if(updateState())
  {                            
        PWM_Calculate();
        Car_Control();     
  }
}

void PWM_Calculate()
{    
  int pwm;
  static int angle_sum = 0;

  angle_sum += Angle_Car;
  angle_sum = constrain(angle_sum, -500, 500);

  pwm = Angle_Car * Ka                    // gain for angle
      + angle_sum * Ki                    // gain for integrated angle
      + Gyro_Car * Kg                     // gain for rotation rate
      + (Speed_Command - Speed_Car) * Ks  // gain for car speed
      + Position_Car * Kp;                // gain for car position

  pwm_r = pwm;
  pwm_l = pwm;

  Serial.print("angle_sum:");
  Serial.print(angle_sum);
  Serial.print(" pwm:");
  Serial.println(pwm);
}

void Car_Control()
{  
 if (pwm_l<0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    pwm_l = -pwm_l;  // change to positive
  }
  else
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
  }
  
  if (pwm_r<0)
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
    pwm_r = -pwm_r;
  }
  else
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
  }
  if( Angle_Car > 45 || Angle_Car < -45 )
  {
    pwm_l = 0;
    pwm_r = 0;
  }
  analogWrite(PWM_L, pwm_l>255? 255 : pwm_l);
  analogWrite(PWM_R, pwm_r>255? 255 : pwm_r);
}

void Encoder_L()   //car up is positive car down is negative
{
  if (digitalRead(SPD_PUL_L))
    encCountLeft += 1;
  else
    encCountLeft -= 1;
}

void Encoder_R()    //car up is positive car down is negative
{
  if (!digitalRead(SPD_PUL_R))
    encCountRight += 1;
  else
    encCountRight -= 1;
}

int updateState()
{

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time

  if(dt >= 0.001)
  {      
    uint8_t temperatureRaw;
    
    /* read IMU */
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    temperatureRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0; // Convert to deg/s : http://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
    double gyroYrate = gyroY / 131.0;
    double temperature = (double)temperatureRaw / 340.0 + 36.53;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
  
    Angle_Car = kalAngleX + angle_zero;
    Gyro_Car = gyroXrate;

    // calculate car velocity and position from encoders
    float avgEncPos = (encCountLeft + encCountRight) * 0.5;
    Position_Car += avgEncPos; 
    Position_Car = constrain(Position_Car, -800, 800);
    Speed_Car = avgEncPos / dt; // speed in encoder counts per second
    
    encCountLeft = 0;
    encCountRight = 0;

    timer = micros();

   return 1;
  }
  return 0;
}

void LEDBlink()
{
  if((millis() - LEDTimer) > 500)
  {
     LEDTimer = millis();
     blinkState = !blinkState;
  }  
}
