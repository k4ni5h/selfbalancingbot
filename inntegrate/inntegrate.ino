/*
  Blink
 
  Turns an LED on for one second, then off for one second, repeatedly.
 
  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products
 
  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman
 
  This example code is in the public domain.
 
  http://www.arduino.cc/en/Tutorial/Blink
*/
//include files for d-v codes
#include "I2Cdev.h"
//include files for angles
#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
MPU6050 mpu;

//v-d constants
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
#define OUTPUT_READABLE_YAWPITCHROLL

//angle contants
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// the setup function runs once when you press reset or power the board
double one_degree = 0.0174532;
double six_degrees = 0.1047192;
double twelve_degrees = 0.2094384;  
double fifty_degrees = 0.87266;
 
const int n_states = 162;         // 3x3x6x3 = 162 states
double alpha = 1000;          // learning rate for action weights
double beta = 0.5;            // learning rate for critic weights
double gamma = 0.95;          // discount factor for critic
double lambda_w = 0.9;    // decay rate for action weights
double lambda_v = 0.8;    // decay rate for critic weights
double max_failures = 50;
double max_steps = 1000000;
 
double max_distance = 2.4;
double max_speed = 1;
double max_angle = 12 * one_degree;
 
double action_weights[n_states] = {};    // action weights
double critic_weights[n_states] = {};    // critic weights
double action_weights_elig[n_states] = {};    // action weight eligibilities
double critic_weights_elig[n_states] = {}; // critic weight eligibilities
 
 
//position, velocity, angle, angle velocity
double x = 0;
double dx = 0;
double t = 0;
double dt = 0;
 
int vd_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}
 
int vd_loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&gx, &gy, &gz);
}

int get_state()
{
    int state = 0;
 
    // failed
    if (x < -max_distance or x > max_distance or t < -max_angle or t > max_angle)
    {
      return(-1);
    }
 
    //position
    if (x < -0.8)
      state = 0;
    else if (x < 0.8)
      state = 1;
    else
      state = 2;
 
    //velocity
    if (dx < -max_speed)
      state += 0;
    else if (dx < 0.5)
      state += 3;
    else
      state += 6;
 
    //angle
    if (t < -six_degrees)
      state += 0;
    else if (t < -one_degree)
      state += 9;
    else if (t < 0)
      state += 18;
    else if (t < one_degree)
      state += 27;
    else if (t < six_degrees)
      state += 36;
    else
      state += 45;
 
    //angle velocity
    if (dt < -fifty_degrees)
      state += 0;
    else if (dt < fifty_degrees)
      state += 54;
    else
      state += 108;
 
    return(state);
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int ang_setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
}


int ang_loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

// read the variables x, dx, t and dt from vrep
int read_variables()
{
//    x = controller.get_current_position()[0];
//    dx = controller.get_current_ground_speed()[0];
//    t = controller.get_current_angle()[1];
//    dt = controller.get_current_angle_speed()[1];
}
 
 
 
  // executes action and updates x, dx, t, dt
  // action size is two) left and right
int do_action(bool action)
{
    //if (action == True)
    //  controller.set_target_velocities(max_speed,max_speed);
    //else
    //  controller.set_target_velocities(-max_speed,-max_speed);
}
 
 
 
  // update all weights or reset them when failed
int update_all_weights(double rhat,bool failed)
{
    for (int i=0; i<n_states; i++)
    {
        action_weights[i] += alpha * rhat * action_weights_elig[i];
        critic_weights[i] += beta * rhat * critic_weights_elig[i];
 
        if (critic_weights[i] < -1.0)
          critic_weights[i] = critic_weights[i];
 
        if (failed == true)
        {
          action_weights_elig[i] = 0;
          critic_weights_elig[i] = 0;
        }
        else
        {
          action_weights_elig[i] = action_weights_elig[i] * lambda_w;
          critic_weights_elig[i] = critic_weights_elig[i] * lambda_v;
        }
    }
   
}
 
 
 
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
 
}
 
// the loop function runs over and over again forever
void loop()
{
 
}
