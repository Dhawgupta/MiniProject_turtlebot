#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
// nodehandle for the ros node
ros::NodeHandle nh;

//std_msgs::UInt16 LeftSpeed; // they ubscribe to the left and right wheel speed
//std_msgs::UInt16 RightSpeed; // right wheel speed subscriber


// target speed variables for the left and right wheel
int16_t LeftTargetSpeed =0;
int16_t RightTargetSpeed = 0;


// callback function for left and right velocities values
void targetspeedLeftcb(const std_msgs::Int16& msg){
  LeftTargetSpeed = msg.data;
  
}
void targetspeedRightcb(const std_msgs::Int16& msg){
  RightTargetSpeed  = msg.data;
}


// standard message to send the left and right encoder values 
//we may require to use UInt64 instead of Float64 that we willl see
std_msgs::Int64 encoderL ;
std_msgs::Int64 encoderR ;

//Subscriber for left and right wheel speeds
ros::Subscriber<std_msgs::Int16> Lspeed("left_wheel_speed",&targetspeedLeftcb);
ros::Subscriber<std_msgs::Int16> Rspeed("right_wheel_speed",&targetspeedRightcb);

// Publisher of odometry data i.e. ticks value.
ros::Publisher Lencoder("left_encoder",&encoderL);
ros::Publisher Rencoder("right_encoder",&encoderR);

/*
float left_wheel_radius ; // assign the left wheel radius
float right_wheel_radius ; // assign the right wheel radius

int ticks_rot ; //ticks per rotation
float base_length ; // the wheel base;
*/
// Arduin Mega has 6 interrupt pins - 2, 3, 18, 19, 20, 21

int left_encoderPin1 = 2;
int left_encoderPin2 = 3;
int right_encoderPin1 = 18;
int right_encoderPin2 = 19;

volatile int lastLencoded =0;
volatile long encoderLvalue = 0;
volatile int lastRencoded =0;
volatile long encoderRvalue = 0;

long lastLeftEncValue = 0;
long lastRightEncValue = 0;

int lastLmsb = 0;
int lastLlsb = 0;
int lastRmsb = 0;
int lastRlsb = 0;

/////////////////////////////////////////////////////////
//defining the motor pins
int l1 =11,l2 = 10,r1 = 9,r2 = 8,lpwm = 6,rpwm = 7;

// setup function
void setup()
{
  nh.initNode();
  nh.advertise(Lencoder);
  nh.advertise(Rencoder);
  nh.subscribe(Lspeed);
  nh.subscribe(Rspeed);

  // motor pins for output
  pinMode(l1,OUTPUT);
  pinMode(l2,OUTPUT);
  pinMode(r1,OUTPUT);
  pinMode(r2,OUTPUT);
  pinMode(lpwm,OUTPUT);
  pinMode(rpwm,OUTPUT);

  
  pinMode(left_encoderPin1,INPUT);
  pinMode(left_encoderPin2,INPUT);
  pinMode(right_encoderPin1,INPUT);
  pinMode(right_encoderPin2,INPUT);
  digitalWrite(left_encoderPin1,HIGH); // turn pull up resistor high for all
  digitalWrite(left_encoderPin2,HIGH);
  digitalWrite(right_encoderPin1,HIGH);
  digitalWrite(right_encoderPin2,HIGH);
  //attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
  attachInterrupt(digitalPinToInterrupt(left_encoderPin1), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoderPin2), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoderPin1), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoderPin2), updateRightEncoder, CHANGE);
  nh.loginfo("Arduino Setup Executed !!!");
}



void loop()
{

  

  //writing the encoder data to the ROS variable
  encoderL.data = encoderLvalue;
  encoderR.data = encoderRvalue;
  
  
 //encoderLvalue, encoderRvalue for the left and right encoder values
 // we write the publiher code to publish the encoder data
 Lencoder.publish(&encoderL);
 Rencoder.publish(&encoderR);
 // so this publishs our encoder values

 // now we will update the motor speeds
 Update_Motors();
 
 
 nh.spinOnce();
 delay(50);
 
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//Code to update the Left and Right Encoder
void updateLeftEncoder()
{
  int MSB = digitalRead(left_encoderPin1);
  int LSB = digitalRead(left_encoderPin2);

  int encoded = (MSB << 1) | LSB; // converting 2 bit to single number
  int sum = (lastLencoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderLvalue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderLvalue --;

  lastLencoded = encoded; // store last cvalue for next time.
  
} 
void updateRightEncoder()
{
  int MSB = digitalRead(right_encoderPin1);
  int LSB = digitalRead(right_encoderPin2);

  int encoded = (MSB << 1) | LSB; // converting 2 bit to single number
  int sum = (lastRencoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderRvalue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderRvalue --;

  lastRencoded = encoded; // store last cvalue for next time.
  
} 



//////////////////////////////////////////////////////////////////////////
// motor running function
// first we define all the variable for the motor pins and pwm pins 


void moveRightMotor(int val)
{
  if (val > 0)
  {
    digitalWrite(r1,HIGH);
    digitalWrite(r2,LOW);
    analogWrite(rpwm,val);
  }
  else if (val < 0)
  {
    digitalWrite(r1,LOW);
    digitalWrite(r2,HIGH);
    analogWrite(rpwm,abs(val));
  }
  else if (val == 0)
  {
    digitalWrite(r1,HIGH);
    digitalWrite(r2,HIGH);
    
  }  
  nh.loginfo("Right Motor Speed Updated to : ");
  nh.loginfo(val);
  
}

void moveLeftMotor(int val)
{
  if (val > 0)
  {
    digitalWrite(l1,HIGH);
    digitalWrite(l2,LOW);
    analogWrite(lpwm,val);
  }
  else if (val < 0)
  {
    digitalWrite(l1,LOW);
    digitalWrite(l2,HIGH);
    analogWrite(lpwm,abs(val));
  }
  else if (val == 0)
  {
    digitalWrite(l1,HIGH);
    digitalWrite(l2,HIGH);
    
  }
  nh.loginfo("Left Motor Speed Updated to : ");
  nh.loginfo(val);
  
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
// Code to  update the motor and their target speed using the variables
// LeftTargetSpeed and RightTargetSpeed ( -255 <= val <= 255) 
void Update_Motors()
{
  moveRightMotor(RightTargetSpeed);
  moveLeftMotor(LeftTargetSpeed);
}

