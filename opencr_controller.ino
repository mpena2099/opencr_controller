#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <synkar_msgs/OdometryCompacted.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh;

#include "protocol_opencr.h"

#define WHEELBASE 0.45f

synkar_msgs::OdometryCompacted odometry1_msg;
synkar_msgs::OdometryCompacted odometry2_msg;
nav_msgs::Odometry odometry_msg;

void callback1(const BaseReceiver & msg) {
  nh.loginfo("callback1");
  odometry1_msg.x = -1 * msg.left_pos;
  odometry1_msg.y = -1 * msg.right_pos;
  odometry1_msg.yaw = msg.yaw;
  odometry1_msg.wl = -1 * msg.left_vel;
  odometry1_msg.wr = -1 * msg.right_vel;
}

void callback2(const BaseReceiver & msg) {
  nh.loginfo("callback2");
  odometry2_msg.x = msg.left_pos;
  odometry2_msg.y = msg.right_pos;
  odometry2_msg.yaw = msg.yaw;
  odometry2_msg.wl = msg.left_vel;
  odometry2_msg.wr = msg.right_vel;
}



BaseTransmitter baseTx1(Serial2);
BaseTransmitter baseTx2(Serial4);

BaseReceiver baseRx1(Serial2, &callback1);
BaseReceiver baseRx2(Serial4, &callback2);

void commandVelocityCallback(const geometry_msgs::Twist & msg) {
  baseTx1.left_velocity =  (float)(-1 * (msg.linear.x - msg.angular.z * WHEELBASE / 2));
  baseTx1.right_velocity = (float)(-1 * (msg.linear.x + msg.angular.z * WHEELBASE / 2));
  baseTx2.left_velocity = (float) (msg.linear.x + msg.angular.z * WHEELBASE / 2);
  baseTx2.right_velocity = (float) (msg.linear.x - msg.angular.z * WHEELBASE / 2);
}

ros::Publisher odometry_msg_pub("odom", &odometry_msg);
//ros::Publisher odometry2_msg_pub("odom2", &odometry2_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &commandVelocityCallback);

void setOdomMsgCovariance(float* cov, float diagonal_value)
{
  for(int i = 0; i < 36; i++) {
    if(i == 0 || i == 7 || i == 14 ||
       i == 21 || i == 28 || i == 35)
      cov[i] = diagonal_value;
    else
      cov[i] = 0;
  }
}


void setup() {
  //  tone(BDPIN_BUZZER, 423, 250);
  delay(250);


  Serial2.begin(57600);
  Serial4.begin(57600);
  delay(1000);
  baseTx1.left_velocity = 0.0;
  baseTx1.right_velocity = 0.0;
  baseTx2.left_velocity = 0.0;
  baseTx2.right_velocity = 0.0;

  setOdomMsgCovariance(odometry_msg.twist.covariance, 0.01);
  odometry_msg.header.frame_id = "odom";
  odometry_msg.child_frame_id = "base_link";
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odometry_msg_pub);

}
int cout = 0;
float time_last_connect = 0;
void loop() {

  // Checa se a placa NAO está conectada ao ROS (rosserial na Jetson)
  if (!nh.connected())
  {
    float time_now = millis() / 1000.0; // segundos
    float elapsed_time = time_now - time_last_connect;


    // Para os motores traseiros e zera o dianteiro
    baseTx1.left_velocity = 0.0;
    baseTx1.right_velocity = 0.0;

    baseTx2.left_velocity = 0.0;
    baseTx2.right_velocity = 0.0;

    if (elapsed_time >= 10)
    {
      //  tone(BDPIN_BUZZER, 1000, 500);
      time_last_connect = time_now;
    }
  }

  cout++;

  if (cout == 50) {
    baseTx1.flush();
    baseTx2.flush();
    if (nh.connected()) {
      //float theta = (((int) round((odometry1_msg.yaw + odometry2_msg.yaw) * 28.6478897565))%360)*0.01745329251; // (round(((yaw1 + yaw2) / 2)*(180/pi)) mod 360) * (pi/180)
      float const theta = (odometry1_msg.yaw + odometry2_msg.yaw) / 4; //0.6309
      odometry_msg.twist.twist.linear.x = (odometry1_msg.wl + odometry1_msg.wr + odometry2_msg.wl + odometry2_msg.wr) / 4;
      odometry_msg.twist.twist.angular.z = (odometry1_msg.wr + odometry2_msg.wr - odometry1_msg.wl - odometry2_msg.wl) / ( 2 * WHEELBASE);
      //odometry_msg.twist.twist.angular.x = left_wheel.velocity;
      //odometry_msg.twist.twist.angular.y = right_wheel.velocity;


      odometry_msg.pose.pose.position.x = (odometry1_msg.x + odometry2_msg.x)/2;
      odometry_msg.pose.pose.position.y = (odometry1_msg.y + odometry2_msg.y)/2;
      odometry_msg.pose.pose.position.z = 0.0;
      odometry_msg.pose.pose.orientation.x = 0.0;
      odometry_msg.pose.pose.orientation.y = 0.0;//theta*2;
      odometry_msg.pose.pose.orientation.z = sin(theta );
      odometry_msg.pose.pose.orientation.w = cos(theta );
      odometry_msg.header.stamp = nh.now();
      odometry_msg_pub.publish(&odometry_msg);
    }
    cout = 0;
    nh.loginfo("-");
  }
  baseRx1.spin();
  baseRx2.spin();


  nh.spinOnce();
  //
  delay(2);
}
