/*
   rosserial Publisher Example
   Prints "hello world!"
*/
//#define USE_USBCON // for atmega32u4
#include <ros.h>
#include <std_msgs/String.h>
#include "ros/node_handle.h"
#include "ArduinoHardware.h"

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
