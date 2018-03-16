#include <Kalman.h>
#include "I2Cdev.h"
#include <MPU6050.h>

#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

double a_x, a_y, a_z;
double g_x, g_y, g_z;

double gyroRoll, gyroPitch;
double accRoll, accPitch;
uint32_t timer;

double Roll, Pitch;
Kalman kalmanRoll;                                                  //create kalman object
Kalman kalmanPitch;
double a[5];
double sum = 0.0;
const int in1 = 7;
const int in2 = 8;
const int pwm1 = 6;
const int in3 = 13;
const int in4 = 12;
const int pwm2 = 10;
float kp = 100, kd = 2.3, ki = 0.0; //1.6,3
float thre_pitch = -5.8;
float correction = 0;
float error = 0, prev_error = 0, diff_error = 0, sum_error = 0;
int final_pwm1 = 0, final_pwm2 = 0;
void setup() {
  //Initializing the data of array to zero.
  for (int i = 0; i < 5; i++)
  {
    a[i] = 0;
  }
//    Serial.begin(38400);


  motor_init();

  //Initial Setup for library
  Wire.begin();
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true) ;
  accelgyro.setSleepEnabled(false);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //initializing timer
  timer = micros();

  //get raw accelerometer gyro and mag data
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  //storing int data to double variable for calculations
  a_x = ax; a_y = ay; a_z = az;
  g_x = gx; g_y = gy; g_z = gz;



  //calculate accelerometer angle in degree
  accRoll  = atan2(a_x, a_z) * (180 / PI);
  accPitch = atan(-a_x / sqrt(a_y * a_y + a_z * a_z)) * (180 / PI);



  //set starting angle for kalman and gyro
  kalmanRoll.setAngle(accRoll);
  kalmanPitch.setAngle(accPitch);
  gyroRoll = accRoll;
  gyroPitch = accPitch;
}

void loop() {
  //get raw accelerometer gyro and mag data
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //storing int data to double variable for calculations
  a_x = ax; a_y = ay; a_z = az;
  g_x = gx; g_y = gy; g_z = gz;

  // Calculate delta time and update timer
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  // Convert to deg/s
  double gyroXrate = g_x / 131.0;
  double gyroYrate = g_y / 131.0; // Convert to deg/s

  // Calculate gyro angle without any filter
  gyroRoll += gyroXrate * dt;
  gyroPitch += gyroYrate * dt;

  //Calculate angles in degree using accelerometer data
  accRoll  = atan2(a_x, a_z) * (180 / PI);
  accPitch = atan(-a_x / sqrt(a_y * a_y + a_z * a_z)) * (180 / PI);



  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accRoll < -90 && Roll > 90) || (accRoll > 90 && Roll < -90))
  {
    kalmanRoll.setAngle(accRoll);
    Roll = accRoll;
    gyroRoll = accRoll;
  }
  else
    Roll = kalmanRoll.getAngle(accRoll, gyroXrate, dt);         // Calculate the angle using a Kalman filter

  if (abs(Roll) > 90)
    gyroYrate = -gyroYrate;                                    // Invert rate, so it fits the restriced accelerometer reading
  Pitch = kalmanPitch.getAngle(accPitch, gyroYrate, dt);

  // Reset the gyro angle when it has drifted too much
  if (gyroRoll < -180 || gyroRoll > 180)
    gyroRoll = Roll;
  if (gyroPitch < -180 || gyroPitch > 180)
    gyroPitch = Pitch;

  //moving average
  for (int i = 4; i >= 1; i--)
  {
    a[i] = a[i - 1];
    a[0] = Pitch;
  }
  for (int i = 0; i < 5; i++)
  {
    sum += a[i];
  }
  sum = sum / 5;
  Serial.println(sum);
  pid();
  self_bal();
  print_data();
}
