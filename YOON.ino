#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

String signs;

#define TXD 2
#define RXD 3

ros::NodeHandle  nh;
SoftwareSerial hc06(TXD, RXD);

void alarm( const std_msgs::String& msg){ 
  signs = msg.data;
    if (signs == "crack"){
      hc06.write("crack");
      delay(5);
    }
    else if (signs == "pothole"){
      hc06.write("pothole");
      delay(5);
    }
    else if (signs == "fight"){
      hc06.write("fight");
      delay(5);
    }
}

ros::Subscriber<std_msgs::String> sub("alarm", &alarm);

void setup() {
  Serial.begin(9600);
  hc06.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}
