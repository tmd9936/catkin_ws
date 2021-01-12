/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

int motorDirPin = 8;
int motorPWMPin = 9;


//void doTurn(bool dir, int vel){
//  digitalWrite(motorDirPin, dir);
//  analogWrite(motorPWMPin, vel);
//}

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  

}

void motor_cb( const std_msgs::UInt16& motor_msg){
  digitalWrite(8,HIGH);
  analogWrite(9,motor_msg.data);

}

ros::Subscriber<std_msgs::UInt16> sub("servo_val", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub2("motor_val", motor_cb);

void setup(){
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  //Serial.begin(9600);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
