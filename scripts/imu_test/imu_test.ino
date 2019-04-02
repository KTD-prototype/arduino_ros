#include <Wire.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <SparkFunLSM9DS1.h>
#include <TimerOne.h>
#include <ros.h>
#include <arduino_ros/imu_customed.h>
#include "ros/node_handle.h"
#include "ArduinoHardware.h"

ros::NodeHandle nh;

arduino_ros::imu_customed imu;
ros::Publisher pubimu("imu/data", &imu);

LSM9DS1 imu_LSM9DS1;
#define LSM9DS1_M 0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6A // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED
//#define PRINT_RAW

// if this flag == 1, then execute initial process to calibrate gyro offset
volatile int interrupt_flag = 1;

float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;
int previous_time = 0;
int present_time = 0;
int passed_time = 0;

float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, roll, pitch, heading, ACCroll, ACCpitch;
float k, g;
#define C 1

// define the number of sample to get data to calibrate and sampling rate
#define NUM_OF_SAMPLES_FOR_INIT 500
#define SAMPLING_RATE 100



void setup() {

  nh.getHardware()->setBaud(9600);

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu_LSM9DS1.settings.device.commInterface = IMU_MODE_I2C;
  imu_LSM9DS1.settings.device.mAddress = LSM9DS1_M;
  imu_LSM9DS1.settings.device.agAddress = LSM9DS1_AG;


  setupGyro();
  setupAccel();
  setupMag();

  // The above lines will only take effect AFTER calling
  // imu_LSM9DS1.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu_LSM9DS1.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }

  // initial process to subtract gyro offset from measured data
  init_gyro_process();
  Serial.println("finished initialization !");
  Serial.println();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);

  Timer1.initialize(1000000 / SAMPLING_RATE); //interrupt per 10000 micro seconds(10 msec)
  Timer1.attachInterrupt(interrupt_function);
}

void loop() {
  if (interrupt_flag == 1) {
    get_IMU_data();
    //    normarize_gyroZ();
    get_posture_complementary_filter();

    imu.gyroX = gyroX;
    imu.gyroY = gyroY;
    imu.gyroZ = gyroZ;
    imu.pitch = pitch;
    imu.roll = roll;
    imu.heading = heading;
    //    print_posture();
    //    print_gyro();
    //    print_accel();
    //    print_time();
    interrupt_flag = 0;
    pubimu.publish(&imu);
  }
}


void interrupt_function() {
  interrupt_flag = 1;
}
