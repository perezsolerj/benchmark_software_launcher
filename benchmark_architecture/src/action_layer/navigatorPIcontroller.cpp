#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "ros/ros.h"

#define SAT	5

//Camera info callback to get size of the camera
class PoseCallback{
  public:
    double pos[3];
    double quat[4];
 
    void callback(const geometry_msgs::PoseStamped& msg) {
      pos[0]=msg.pose.position.x;
      pos[1]=msg.pose.position.y;
      pos[2]=msg.pose.position.z;

      quat[0]=msg.pose.orientation.x;
      quat[1]=msg.pose.orientation.y;
      quat[2]=msg.pose.orientation.z;
      quat[3]=msg.pose.orientation.w;
    }
};

class BenchmarkInfoCallback{
  public:
    int newIteration;

    BenchmarkInfoCallback(){
      newIteration=0;
    }

    void callback(const std_msgs::String& msg) {
      if(msg.data!="")
	newIteration=1;
    }
};

int main(int argc, char **argv){
  std::string pose_topic, velocity_topic, benchinfo_topic;
  PoseCallback pose;
  BenchmarkInfoCallback benchInfo;
  double gain,Igain;

  ros::init(argc, argv, "navigatorPIcontroller");
  ros::NodeHandle nh;
 
  nh.param("pose", pose_topic, (std::string)"gotopose");
  nh.param("velocity", velocity_topic, (std::string)"dataNavigator");
  nh.param("benchinfo", benchinfo_topic, (std::string)"BenchmarkInfo");
  nh.param("gain", gain, 0.5);
  nh.param("Igain", Igain, 0.001);

  ros::Subscriber sub_pose = nh.subscribe(pose_topic, 1000, &PoseCallback::callback,&pose);
  ros::Subscriber sub_benchInfo = nh.subscribe(benchinfo_topic, 1000, &BenchmarkInfoCallback::callback,&benchInfo);
  ros::Publisher pub=nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 1);

  double Itermx=0;
  double Itermy=0;
  double Itermz=0;

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    //Check if newIteration started to restart Iterms
    if(benchInfo.newIteration){
      Itermx=0;
      Itermy=0;
      Itermz=0;
      benchInfo.newIteration=0;
    }
	
    //Compute control law

    double errorx=gain*(pose.pos[0]);
    double errory=gain*(pose.pos[1]);
    double errorz=gain*(pose.pos[2]);


    Itermx+=pose.pos[0];
    Itermy+=pose.pos[1];
    Itermz+=pose.pos[2];

    //Send message to Simulator
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x=errorx+Igain*Itermx;
    msg.twist.linear.y=errory+Igain*Itermy;
    msg.twist.linear.z=errorz+Igain*Itermz;
    msg.twist.angular.x=0;
    msg.twist.angular.y=0;
    msg.twist.angular.z=0;

    if (msg.twist.linear.x>SAT) msg.twist.linear.x=SAT; else if (msg.twist.linear.x<-SAT) msg.twist.linear.x=-SAT;
    if (msg.twist.linear.y>SAT) msg.twist.linear.y=SAT; else if (msg.twist.linear.y<-SAT) msg.twist.linear.y=-SAT;
    if (msg.twist.linear.z>SAT) msg.twist.linear.z=SAT; else if (msg.twist.linear.z<-SAT) msg.twist.linear.z=-SAT;

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

}

