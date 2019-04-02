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

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);
}

void loop() {
  // put your main code here, to run repeatedly:

}
