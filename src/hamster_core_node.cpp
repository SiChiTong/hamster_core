/* Authors: Baram */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "hamster_core/sensor.h"

#include "hamster.h"


Hamster hamster;
hamster_core::sensor sensor_msg;


double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;
uint32_t motor_pre_time;

void updateSensor();
void updateMotor();
void cmdVelCallback(const geometry_msgs::Twist& msg);


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hamster_core_node");
  ros::NodeHandle nh;  
  ros::Rate loop_rate(100);

  ros::Publisher  sensor_pub = nh.advertise<hamster_core::sensor>("hamster/sensor", 1000);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel",1000, &cmdVelCallback);


  motor_pre_time = millis();

  if (argc >= 2)
  {
    if (hamster.begin(argv[1]) == true)
    {
      printf("Hamster Begin OK\n");
    }
    else
    {
      exit(1);
    }
  }
  else
  {
    printf("Need Serial Port\n");
    exit(1);
  }


  while (ros::ok())
  {
    updateSensor();
    updateMotor();
    sensor_pub.publish(sensor_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void updateSensor()
{
  sensor_msg.left_floor = hamster.leftFloor();
  sensor_msg.right_floor = hamster.rightFloor();
  sensor_msg.left_proximity = hamster.leftProximity();
  sensor_msg.right_proximity = hamster.rightProximity();
  sensor_msg.acceleration_x = hamster.accelerationX();
  sensor_msg.acceleration_y = hamster.accelerationY();
  sensor_msg.acceleration_z = hamster.accelerationZ();
  sensor_msg.light = hamster.light();
  sensor_msg.temperature = hamster.temperature();
  sensor_msg.battery = hamster.battery();
}

void updateMotor()
{
  double lin_vel_left;
  double lin_vel_right;


  lin_vel_left  = goal_linear_velocity - (goal_angular_velocity/2);
  lin_vel_right = goal_linear_velocity + (goal_angular_velocity/2);
    

  lin_vel_left  = constrain(lin_vel_left  * 100, -100, 100);
  lin_vel_right = constrain(lin_vel_right * 100, -100, 100);
  
  hamster.wheels(lin_vel_left, lin_vel_right);

  
  if (millis()-motor_pre_time > 1000)
  {
    hamster.wheels(0, 0);
  }  
}

void cmdVelCallback(const geometry_msgs::Twist& msg)
{
  goal_linear_velocity  = msg.linear.x;
  goal_angular_velocity = msg.angular.z;
  motor_pre_time = millis();
}