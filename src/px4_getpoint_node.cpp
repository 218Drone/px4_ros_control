#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <keyboard/Key.h>
#include <math.h>
#include <angles/angles.h>
// fixed
 #include </home/joel/catkin_ws/devel_isolated/v4r_ellipses/include/v4r_ellipses/center.h>

//set velocity
ros::Publisher vel_sp_pub;
//current position 
// next position
geometry_msgs::PoseStamped currentPos,nextPos,initCirclePos;
geometry_msgs::TwistStamped vs;
sensor_msgs::NavSatFix currentGlobalPos;
v4r_ellipses::center findEllipseMsg;
bool isGetpoint, isGetGPSPoint, isSearch;
float speed;
//radius
int R = 6371000;
//FOV
float camera_hfov = 72.42;
float camera_vfov  = 43.3;
int camera_width = 640;
int camera_height = 480;
float x,y,radius;
float originX, originY;
float positionX, positionY;
float targetHeading;
float targetDistance;
float speedSim;
int id;
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

//GPS point
void gotoGPSPoint(double lat, double lon){
    double delta_x = R * (lat - currentGlobalPos.latitude) * float(3.1415926535898 / 180);
    double delta_y = R * (lon - currentGlobalPos.longitude) * float(3.1415926535898 / 180);
    vs.twist.linear.x = delta_x*0.1;
    vs.twist.linear.y = -delta_y*0.1;
    vs.header.stamp = ros::Time::now();
    vel_sp_pub.publish(vs);
}
// calculate position X Y 
void pixelPointToPositionXY(float x, float y, float alt){
    float thetaX = x*camera_hfov/camera_width;
    float thetaY = y*camera_vfov/camera_height;
    positionX =  alt*tan(thetaX*3.1415926/180);
    positionY = alt*tan(thetaY*3.1415926/180);
}
void shiftToOrigin(float x, float y, int camera_width, int camera_height){
  // return (x+camera_width/2)
  originX = x - camera_width/2.0;
  originY = -1.0*y + camera_height/2.0;
  ROS_INFO_STREAM("originX:" << originX);
  ROS_INFO_STREAM("originY:" << originY);
}

// search target 
void searchTarget(){
    shiftToOrigin(x, y, camera_width,camera_height);
    // wait to fixed!!!  degree, not radians  attitude.roll
    //have issues!!!!
    // originX = originX - (camera_width/camera_hfov)*(30);
    // originY = originY - (camera_height/camera_vfov)*(30);
    
    // convert to distance  currentGlobalPos.altitude
    pixelPointToPositionXY(originX,originY,1.0);
    // convert to world coordinates
    // to be fixed!! radians 
    // targetHeading = atan(positionY/positionX)%(2*3.1415926);// to be fixed
    targetHeading = atan(positionY/positionX);
    //attitude.yaw 
    targetHeading =  1.0 - targetHeading;
    targetDistance = sqrt(positionX*positionX + positionY*positionY);

    ROS_INFO_STREAM("targetDistance:" << targetDistance);
    // calculate speed toward target
    speedSim = targetDistance * 0.02;
    // calculate cartisian speed 
    // to be fixed!!
    ROS_INFO_STREAM("speedSim:" << speedSim);

}

//received local position
void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg){
    currentPos = *msg;
}

void globalPositionReceived(const sensor_msgs::NavSatFixConstPtr& msg){
   currentGlobalPos = *msg;
}
void findEllipsesReceived(const v4r_ellipses::center& msg ){
  findEllipseMsg = msg;
  id=findEllipseMsg.id;
  x=findEllipseMsg.center[0];
  y=findEllipseMsg.center[1];
  radius=findEllipseMsg.radius;
  ROS_INFO_STREAM("id:" << id);
  ROS_INFO_STREAM("centerX:" << x);
  ROS_INFO_STREAM("centerY:" << y);
  ROS_INFO_STREAM("radius:" << radius);
}
//sub attitude 
void attitudeReceived(){

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
      	isGetGPSPoint = false;
      isSearch=false;
      	ROS_INFO_STREAM("get to point 10 10 3");
              break;
      }
      //GPS point
      case 'g':
      {
      	isGetGPSPoint = true;
      	isGetpoint = false;
      isSearch=false;
      	ROS_INFO_STREAM("get to GPS point");
      	break;
      }
      //case search plan q
      case 'q':
      {
        isGetpoint=false;
        isGetGPSPoint=false;
        isSearch=true;
        ROS_INFO_STREAM("get to search target");
        break;
      }

      case 'h':
      {
        // turn to manual mode
        isGetpoint = false;
        isGetGPSPoint =false;
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
  ros::Subscriber findEllipsesSubsciber = nodeHandle.subscribe("/find_ellipse", 10, findEllipsesReceived);
  // ros::Subscriber attitudeSubsciber =  nodeHandle.subscribe("/mavros/global_position/raw/fix", 10, attitudeReceived);

  isGetpoint = false;
  isGetGPSPoint = false;
  isSearch=false;
  speed = 0.2;
//ros rate 
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {

      vs.header.seq++;
      //GPS topic 
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.longitude);
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.latitude);
      // ROS_INFO_STREAM("GPS longitude" << currentGlobalPos.altitude);

      if((!isGetpoint) && (!isGetGPSPoint)&&(!isSearch)){

        vs.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM("send ps" << ps);
        vel_sp_pub.publish(vs);
      }else if(isGetGPSPoint) {
         gotoGPSPoint(55.7530147,37.6273584);
         ROS_INFO_STREAM("GPS");
      }else if(isGetpoint){
          getPoint(10,10,3);
          ROS_INFO_STREAM("local position");
      }else if(isSearch){
        searchTarget();
        ROS_INFO_STREAM("searching!!!");
      }
    ros::spinOnce();

    loop_rate.sleep();
  }
}
