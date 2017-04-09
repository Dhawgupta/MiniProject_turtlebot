// Using Acceleromeer ADXL 335
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <rosserial_arduino/Adc.h>

const int xpin = A2;  //x-axis accelerometer
const int ypin = A1;  // y- axis pin
const int zpin = A0; // z axis pin

ros::NodeHandle nh;

//creating an adc message
rosserial_arduino::Adc adc_msg;

ros::Publisher pub("adc",&adc_msg);

void setup()
{
  nh.initNode();
  nh.advertise(pub);
  
}

// we average the analog readinto remove the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0;i<4;i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{
 // inserting ADC values to ADC Message
 adc_msg.adc0 = averageAnalog(xpin);
 adc_msg.adc1 = averageAnalog(ypin);
 adc_msg.adc2 = averageAnalog(zpin);

 pub.publish(&adc_msg);

 nh.spinOnce();

 delay(10);
 
}

