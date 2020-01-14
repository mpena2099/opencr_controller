#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
//#include <std_srvs/Empty.h>
//#include <std_srvs/Trigger.h>
//#include <synkar_base_controller/BatStatus.h>
//#include <synkar_base_controller/BoardStatus.h>
//#include <node_alive/Watchdog.h>

ros::NodeHandle nh;

#include "protocol_opencr.h"

#define WHEELBASE 0.45f
#define TIME_THRESHOLD 200 //milliseconds
struct OdometryCompacted
{
  float x;
  float y;
  float yaw;
  float wl;
  float wr;
};
OdometryCompacted odometry1_msg;
OdometryCompacted odometry2_msg;
nav_msgs::Odometry odometry_msg;
uint16_t battery_voltage;

unsigned long time1_last = 0;
unsigned long time2_last = 0;
bool active_status_1 = false;
bool active_status_2 = false;

void callback1(const BaseReceiver &msg)
{
  //nh.loginfo("cb1");
  odometry1_msg.x = -1 * msg.left_pos;
  odometry1_msg.y = -1 * msg.right_pos;
  odometry1_msg.yaw = msg.yaw;
  odometry1_msg.wl = -1 * msg.left_vel;
  odometry1_msg.wr = -1 * msg.right_vel;
  time1_last = millis();
}

void callback2(const BaseReceiver &msg)
{
  //nh.loginfo("cb2");
  odometry2_msg.x = msg.left_pos;
  odometry2_msg.y = msg.right_pos;
  odometry2_msg.yaw = msg.yaw;
  odometry2_msg.wl = msg.left_vel;
  odometry2_msg.wr = msg.right_vel;
  battery_voltage = msg.bat_vel;

  time2_last = millis();
}
void time_spin()
{
  long int time1_current = millis();
  active_status_1 = ((time1_current - time1_last) < TIME_THRESHOLD);

  unsigned long time2_current = millis();
  active_status_2 = ((time2_current - time2_last) < TIME_THRESHOLD);
}

BaseTransmitter baseTx1(Serial2);
BaseTransmitter baseTx2(Serial3);

BaseReceiver baseRx1(Serial2, &callback1);
BaseReceiver baseRx2(Serial3, &callback2);

unsigned long time_ros_last = 0;

void commandVelocityCallback(const geometry_msgs::Twist &msg)
{
  baseTx1.left_velocity = (float)(-1 * (msg.linear.x - msg.angular.z * WHEELBASE / 2));
  baseTx1.right_velocity = (float)(-1 * (msg.linear.x + msg.angular.z * WHEELBASE / 2));
  baseTx2.left_velocity = (float)(msg.linear.x + msg.angular.z * WHEELBASE / 2);
  baseTx2.right_velocity = (float)(msg.linear.x - msg.angular.z * WHEELBASE / 2);
  time_ros_last = millis();
}
/*
void bat_cb(const synkar_base_controller::BatStatusRequest &req, synkar_base_controller::BatStatusResponse &res)
{
  res.value = ((float)battery_voltage) / 10.0f;
}

void board1_cb(const synkar_base_controller::BoardStatusRequest &req, synkar_base_controller::BoardStatusResponse &res)
{
  res.success = baseRx1.receive_success_msgs;
  res.fail = baseRx1.receive_fail_msgs;
  res.quality = (float)baseRx1.receive_success_msgs / (baseRx1.receive_fail_msgs + baseRx1.receive_success_msgs);
  res.active = active_status_1;
}

void board2_cb(const synkar_base_controller::BoardStatusRequest &req, synkar_base_controller::BoardStatusResponse &res)
{
  res.success = baseRx2.receive_success_msgs;
  res.fail = baseRx2.receive_fail_msgs;
  res.quality = (float)baseRx2.receive_success_msgs / (baseRx2.receive_fail_msgs + baseRx2.receive_success_msgs);
  res.active = active_status_2;
}

char battery_buff[8];
char b1_quality_buff[8];
char b2_quality_buff[8];
/*
void watchdog_cb(node_alive::WatchdogRequest& req, node_alive::WatchdogResponse & res) {
  
  dtostrf((((float) battery_voltage) / 10.0f), 4, 1, battery_buff);
  if((baseRx1.receive_fail_msgs + baseRx1.receive_success_msgs) > 0)
    dtostrf(((float) baseRx1.receive_success_msgs/(baseRx1.receive_fail_msgs + baseRx1.receive_success_msgs)), 5, 3, b1_quality_buff);
  else
    dtostrf(0.0, 5, 3, b1_quality_buff);
  if((baseRx2.receive_fail_msgs + baseRx2.receive_success_msgs) > 0)
    dtostrf(((float) baseRx2.receive_success_msgs/(baseRx2.receive_fail_msgs + baseRx2.receive_success_msgs)), 5, 3, b2_quality_buff);
  else
    dtostrf(0.0, 5, 3, b2_quality_buff);
  diagnostic_msgs::KeyValue * dummy = new diagnostic_msgs::KeyValue[6];
  dummy[0].key = "Board 1 - battery";
  dummy[0].value = const_cast<char*>(battery_buff);
  dummy[1].key = "Board 1 - quality";
  dummy[1].value = const_cast<char*>(b1_quality_buff);
  dummy[2].key = "Board 1 - active";
  if(active_status_1)
    dummy[2].value = "true";
  else
    dummy[2].value = "false";
  dummy[3].key = "Board 2 - battery";
  dummy[3].value = const_cast<char*>(battery_buff);
  dummy[4].key = "Board 2 - quality";
  dummy[4].value = const_cast<char*>(b2_quality_buff);
  dummy[5].key = "Board 2 - active";
  if(active_status_2)
    dummy[5].value = "true";
  else
    dummy[5].value = "false";
      
  res.watched_values = dummy;
  res.watched_values_length = 6;
}*/

ros::Publisher odometry_msg_pub("odom", &odometry_msg);
//ros::Publisher odometry2_msg_pub("odom2", &odometry2_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &commandVelocityCallback);
//ros::Publisher bat_vel_pub("bat_vel", &battery_voltage);
/*ros::ServiceServer<synkar_base_controller::BatStatusRequest, synkar_base_controller::BatStatusResponse> bat_service("~battery", &bat_cb);
ros::ServiceServer<synkar_base_controller::BoardStatusRequest, synkar_base_controller::BoardStatusResponse> board1_service("~board1", &board1_cb);
ros::ServiceServer<synkar_base_controller::BoardStatusRequest, synkar_base_controller::BoardStatusResponse> board2_service("~board2", &board2_cb);
//ros::ServiceServer<node_alive::WatchdogRequest, node_alive::WatchdogResponse> watchdog_service("~watchdog_srv", watchdog_cb);
*/
void setOdomMsgCovariance(float *cov, float diagonal_value)
{
  for (int i = 0; i < 36; i++)
  {
    if (i == 0 || i == 7 || i == 14 ||
        i == 21 || i == 28 || i == 35)
      cov[i] = diagonal_value;
    else
      cov[i] = 0;
  }
}
uint32_t chave_time_pressed = 0;
bool chave_status_pressed = false;

void chave_cb(const std_msgs::Empty &msg)
{
  digitalWrite(4, LOW);
  chave_status_pressed = true;
  chave_time_pressed = millis();
}
ros::Subscriber<std_msgs::Empty> chave_sub("key_activate", &chave_cb);
std_msgs::Bool chave_status_msg;
ros::Publisher chave_status_pub("key_status", &chave_status_msg);
/*void chave_cb(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  digitalWrite(4, LOW);
  chave_status_pressed = true;
  chave_time_pressed = millis();
}
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> chave_service("~key_activate", &chave_cb);

void chave_status_cb(const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = !digitalRead(3);
  res.message = !res.success ? "opened" : "closed";
}
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse> chave_status_service("~get_status_key", &chave_status_cb);*/
void setup()
{
  //  tone(BDPIN_BUZZER, 423, 250);
  pinMode(50, OUTPUT);
  digitalWrite(50, LOW);
  delay(250);
  digitalWrite(50, HIGH);
  delay(500);
  digitalWrite(50, LOW);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(3, INPUT_PULLUP);
  Serial.begin(115200);
  Serial3.begin(57600);
  Serial2.begin(57600);
  delay(1000);
  baseTx1.left_velocity = 0.0;
  baseTx1.right_velocity = 0.0;
  baseTx2.left_velocity = 0.0;
  baseTx2.right_velocity = 0.0;

  setOdomMsgCovariance(odometry_msg.twist.covariance, 0.01);
  odometry_msg.header.frame_id = "odom";
  odometry_msg.child_frame_id = "synkar_base_link";

  nh.initNode();
  //nh.subscribe(cmd_vel_sub);
  //nh.subscribe(chave_sub);
  nh.advertise(odometry_msg_pub);
  nh.advertise(chave_status_pub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(chave_sub);
  // nh.advertise(bat_vel_pub);
  /*nh.advertiseService<synkar_base_controller::BatStatusRequest, synkar_base_controller::BatStatusResponse>(bat_service);

  nh.advertiseService<synkar_base_controller::BoardStatusRequest, synkar_base_controller::BoardStatusResponse>(board1_service);
  nh.advertiseService<synkar_base_controller::BoardStatusRequest, synkar_base_controller::BoardStatusResponse>(board2_service);
  //nh.advertiseService<node_alive::WatchdogRequest, node_alive::WatchdogResponse>(watchdog_service);
  nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(chave_service);
  nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(chave_status_service);*/
}
int cout = 0;
float time_last_connect = 0;
uint32_t key_status_pub_time = 0;
void loop()
{
  uint32_t loop_time = millis();
  if (chave_status_pressed && millis() - chave_time_pressed > 500)
  {
    digitalWrite(4, HIGH);
    chave_status_pressed = false;
  }

  if (nh.connected() && loop_time - key_status_pub_time > 1000)
  {
   key_status_pub_time = loop_time;
    chave_status_msg.data = !digitalRead(3);
   chave_status_pub.publish(&chave_status_msg);
  }

  // Checa se a placa NAO está conectada ao ROS (rosserial na Jetson)
  if (!nh.connected())
  {
    float time_now = millis() / 1000.0; // segundos
    float elapsed_time = time_now - time_last_connect;
    nh.spinOnce();

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

  if (cout == 20)
  {
    if (millis() - time_ros_last > 500)
    {
      baseTx1.left_velocity = 0.0;
      baseTx1.right_velocity = 0.0;

      baseTx2.left_velocity = 0.0;
      baseTx2.right_velocity = 0.0;
    }
    baseTx1.flush();
    baseTx2.flush();
    if (nh.connected())
    {
      //float theta = (((int) round((odometry1_msg.yaw + odometry2_msg.yaw) * 28.6478897565))%360)*0.01745329251; // (round(((yaw1 + yaw2) / 2)*(180/pi)) mod 360) * (pi/180)
      float const theta = (odometry1_msg.yaw + odometry2_msg.yaw) / 4; //0.6309
      odometry_msg.twist.twist.linear.x = (odometry1_msg.wl + odometry1_msg.wr + odometry2_msg.wl + odometry2_msg.wr) / 4;
      odometry_msg.twist.twist.angular.z = (odometry1_msg.wr - odometry2_msg.wr - odometry1_msg.wl + odometry2_msg.wl) / (2 * 0.45);
      //odometry_msg.twist.twist.angular.x = odometry1_msg.wr;
      //odometry_msg.twist.twist.angular.y = odometry2_msg.wr;

      odometry_msg.pose.pose.position.x = (odometry1_msg.x + odometry2_msg.x) / 2;
      odometry_msg.pose.pose.position.y = (odometry1_msg.y + odometry2_msg.y) / 2;
      odometry_msg.pose.pose.position.z = 0.0;
      odometry_msg.pose.pose.orientation.x = 0.0;
      odometry_msg.pose.pose.orientation.y = 0.0; //theta*2;
      odometry_msg.pose.pose.orientation.z = sin(theta);
      odometry_msg.pose.pose.orientation.w = cos(theta);
      odometry_msg.header.stamp = nh.now();
      odometry_msg_pub.publish(&odometry_msg);
    }
    cout = 0;
    time_spin();
  }
  baseRx1.spin();
  baseRx2.spin();

  nh.spinOnce();
  //
  delay(2);
}
