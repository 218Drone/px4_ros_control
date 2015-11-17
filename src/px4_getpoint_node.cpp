#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <keyboard/Key.h>
#include <math.h>
#include <angles/angles.h>
//set velocity
ros::Publisher vel_sp_pub;
//current position 
// next position
geometry_msgs::PoseStamped currentPos,nextPos,initCirclePos;
geometry_msgs::TwistStamped vs;
sensor_msgs::NavSatFix currentGlobalPos;
bool isGetpoint;
float speed;


// go to specific point 
void getPoint(double x, double y,double z){
    bool reached = false;
    double dis = sqrt((currentPos.pose.position.x-x)*(currentPos.pose.position.x-x) +(currentPos.pose.position.y-y)*(currentPos.pose.position.y-y)  + (currentPos.pose.position.z-z)*(currentPos.pose.position.z-z));
    double thresh = 0.5;
    if(dis < thresh){
    	reached = true;
    	isGetpoint =false;
    	ROS_INFO_STREAM("get There!");
    }

    geometry_msgs::Point velocityV;
    velocityV.x = x - currentPos.pose.position.x;
    velocityV.y = y - currentPos.pose.position.y;
    velocityV.z = z - currentPos.pose.position.z;

    //set velocity
    vs.twist.linear.x = velocityV.x*0.2;
    vs.twist.linear.y = velocityV.y*0.2;
    vs.twist.linear.z = velocityV.z*0.2;
    // ROS_INFO_STREAM("getPoint1");
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
}
//received local position
void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos = *msg;
}

void globalPositionReceived(const sensor_msgs::NavSatFixConstPtr& msg){
   currentGlobalPos = *msg;
}
//keyboard command 
void sendCommand(const keyboard::Key &key){
   switch (key.code)
   {
   		case 'w':
   		{
   			ROS_INFO_STREAM("up");
   			vs.twist.linear.z += 0.1;
   			break;
   		}
      case 's':
      {
        ROS_INFO_STREAM("down");
        vs.twist.linear.z -= 0.1;
        break;
      }
   		case 'j':
   		{
   			ROS_INFO_STREAM("left");
   			vs.twist.linear.y += 0.1;
   			break;
   		}
   		case 'l':
   		{
   			ROS_INFO_STREAM("right");
   			vs.twist.linear.y -= 0.1;
   			break;
   		}
      case 'i':
      {
        ROS_INFO_STREAM("forward");
        vs.twist.linear.x += 0.1;
        break;
      }
      case 'k':
      {
        ROS_INFO_STREAM("backward");
        vs.twist.linear.x -= 0.1;
        break;
      }
      case 'u':
      {
        ROS_INFO_STREAM("rotate left");
        vs.twist.angular.z += 0.1;
        break;
      }
      case 'o':
      {
        ROS_INFO_STREAM("rotate right");
        vs.twist.angular.z -= 0.1;
        break;
      }
      case 'p':
      {
      	isGetpoint = true;
      	ROS_INFO_STREAM("get to point 10 10 3");
              break;
      }
      case 'h':
      {
        // turn to manual mode
        isGetpoint = false;
        vs.twist.linear.x = 0;
        vs.twist.linear.y = 0;
        vs.twist.linear.z = 0;
        vs.twist.angular.z = 0;
        ROS_INFO_STREAM("Manual Mode");
        break;
      }
      
      case 'a':
      {
        speed += 0.1;
        ROS_INFO_STREAM("speed up" << speed);
        break;
      }
      case 'd':
      {
        speed -= 0.1;
        ROS_INFO_STREAM("speed down" << speed);
        break;
      }

   		
   		default:
   		{
   			
   		}

   }
}
int main(int argc, char **argv){
  ros::init(argc, argv, "px4_offboard_velocity_control_node");
  ros::NodeHandle nodeHandle;
  //publish set_velocity topic 100ms
  vel_sp_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
  ros::Subscriber localPositionSubsciber = nodeHandle.subscribe("/mavros/local_position/local", 10, localPositionReceived);
  ros::Subscriber globalPositionSubsciber =  nodeHandle.subscribe("/mavros/global_position/raw/fix", 10, globalPositionReceived);
  isGetpoint = false;
  speed = 0.2;
//ros rate 
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {

      vs.header.seq++;
      //GPS topic 
      ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.longitude);
      ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.latitude);
      ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.altitude);

      if(!isGetpoint){

        vs.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
        vel_sp_pub.publish(vs);
      } else if(isGetpoint){
        getPoint(10,10,3);
        // ROS_INFO_STREAM("getpoint");
      }
    ros::spinOnce();

    loop_rate.sleep();
  }
}
