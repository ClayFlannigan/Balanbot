
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

/* Contants */
const double Kp = 50.0;           // proportial gain for tilt angle
const double Kd = 3.5;            // derivative gain for tilt rate
const double Ki = 0.1;            // integral gain for tilt angle
const double Ks = 0.0;            // proportional gain for car velocity based on encoder counts
const double Kl = 0.0;            // proportional gain for car location based on encoder counts
const double maxIntegrator = 500; // maximum integrator accumulator
const double angle_zero = -2.0;   // angular offset between the sensor zero tilt and the balance point of the car
const double updateRate = 0.004;  // time between state/control updates

typedef struct State {
  int encCountL = 0;              // raw encoder counts
  int encCountR = 0;              // raw encoder counts
  int location = 0;               // position of car in encoder counts
  double angle;                   // tilt of car in degrees
  double gyro;                    // tilt rate of car degrees per second
  double vel;                     // velocity of car in encoder counts per second
  double temp;                    // temperature in degrees  
  Kalman kalman;                  // Kalman filter for angle
} State;

typedef struct Command {
  double vel = 0;                 // commanded speed
  double turn = 0;                // commanded turn rate
  int pwmL = 0;                   // raw motor command
  int pwmR = 0;                   // raw motor command
} Command;

State s;
Command c;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT); 

  // init IMU
  uint8_t i2cData[14];              
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

  // encoder interrupts
  PCintPort::attachInterrupt(SPD_INT_L, Encoder_L,FALLING);
  PCintPort::attachInterrupt(SPD_INT_R, Encoder_R,FALLING);

  delay(200); // Wait for sensor to stabilize
  
  updateState(true);
}

void loop() 
{
  if(updateState(false))
  {                            
        updatePWM();
        commandMotors();     
  }
}

void updatePWM()
{    
  int pwm;
  static int angleSum = 0;

  angleSum += s.angle;
  angleSum = constrain(angleSum, -maxIntegrator, maxIntegrator);

  pwm = s.angle * Kp                  // gain for angle
      + s.gyro * Kd                   // gain for rotation rate
      + angleSum * Ki                 // gain for integrated angle
      + (c.vel - s.vel) * Ks          // gain for car speed
      + s.location * Kl;              // gain for car position

  c.pwmL = pwm + c.turn;
  c.pwmR = pwm - c.turn;

  Serial.print("angleSum:");
  Serial.print(angleSum);
  Serial.print(" pwm:");
  Serial.println(pwm);
}

void commandMotors()
{
  uint8_t cmdL, cmdR;
    
  if (c.pwmL < 0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    cmdL = c.pwmL < -255 ? 255 : -c.pwmL;
  }
  else
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
    cmdL = c.pwmL > 255 ? 255 : c.pwmL;
  }
    
  if (c.pwmR < 0)
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
    cmdR = c.pwmR < -255 ? 255 : -c.pwmR;
  }
  else
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
    cmdR = c.pwmR > 255 ? 255 : c.pwmR;
  }
  if(s.angle > 45 || s.angle < -45)
  {
    cmdR = 0;
    cmdL = 0;
  }
  analogWrite(PWM_L, cmdL);
  analogWrite(PWM_R, cmdR);
}

void Encoder_L()   //car up is positive car down is negative
{
  if (digitalRead(SPD_PUL_L))
    s.encCountL += 1;
  else
    s.encCountL -= 1;
}

void Encoder_R()    //car up is positive car down is negative
{
  if (!digitalRead(SPD_PUL_R))
    s.encCountR += 1;
  else
    s.encCountR -= 1;
}

int updateState(bool init)
{
  static uint32_t timer = 0;        
  double kalAngle;
  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;
  uint8_t i2cData[14];
  uint8_t temperatureRaw; 
  double roll, pitch;             
  double dt = (double)(micros() - timer) / 1000000; // time since last update

  if(dt >= updateRate || init) // 4ms updates
  {      

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
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    s.gyro = gyroX / 131.0; // Convert to deg/s : http://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
    s.temp = temperatureRaw / 340.0 + 36.53;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngle > 90) || (roll > 90 && kalAngle < -90) || init) {
      s.kalman.setAngle(roll);
      kalAngle = roll;
    } else
      kalAngle = s.kalman.getAngle(roll, s.gyro, dt); // Calculate the angle using a Kalman filter
      
    s.angle = kalAngle + angle_zero;

    // calculate car velocity and position from encoders
    s.location += (s.encCountL + s.encCountR) / 2; 
    s.location = constrain(s.location, -1000, 1000);
    s.vel = (s.encCountL + s.encCountR) / 2 / dt; // speed in encoder counts per second
    
    s.encCountL = 0;
    s.encCountR = 0;

    timer = micros();

   return 1;
  }
  return 0;
}
