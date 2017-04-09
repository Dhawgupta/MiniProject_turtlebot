#include <ros.h>

/* nodehandle comes before the setup funciton and 
 *  we can cret only on nodehandle for each serial device
 *  */
ros::NodeHandle nh; 
// Ros Time functions
// ros::Time begin = nh.now();
// double secs = nh.now().toSec();
// ros::Duration ten_seconds(10,0);


void setup()
{
  //initailize the node
nh.initNode();
// nh.advertise(chatter);
// nh.subscribe(sub);
/*
 * nh.logdebug("Debug Statement");
nh.loginfo("Program info");
nh.logwarn("Warnings.);
nh.logerror("Errors..");
nh.logfatal("Fatalities!");
 loggig apis
 */
}
void loop()
{
  //Ros callback once
nh.spinOnce();
}
/* Subcriber and Publsher syntax
 *  ros::Subscriber<std_msgs::String> sub("talker", callback);
 *  ros::Publisher chatter("chatter", &str_msg);
void callback ( const std_msgs::String& msg){
str_msg.data = msg.data;
chatter.publish( &str_msg );
}
 */
 
