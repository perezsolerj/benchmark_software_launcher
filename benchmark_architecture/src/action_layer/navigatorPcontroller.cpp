#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
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

int main(int argc, char **argv){
  std::string pose_topic, velocity_topic;
  PoseCallback pose;
  double gain;

  ros::init(argc, argv, "navigatorPcontroller");
  ros::NodeHandle nh;
 
  nh.param("pose", pose_topic, (std::string)"gotopose");
  nh.param("velocity", velocity_topic, (std::string)"dataNavigator");
  nh.param("gain", gain, 0.5	);

  ros::Subscriber sub_pose = nh.subscribe(pose_topic, 1000, &PoseCallback::callback,&pose);
  ros::Publisher pub=nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 1);

  while(ros::ok())
  {
    ros::spinOnce();
    //Compute control law

    double errorx=gain*(pose.pos[0]);
    double errory=gain*(pose.pos[1]);
    if (errorx>SAT) errorx=SAT; else if (errorx<-SAT) errorx=-SAT;
    if (errory>SAT) errory=SAT; else if (errory<-SAT) errory=-SAT;

    //Send message to Simulator
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x=errorx;
    msg.twist.linear.y=errory;
    msg.twist.linear.z=0;
    msg.twist.angular.x=0;
    msg.twist.angular.y=0;
    msg.twist.angular.z=0;

    if (msg.twist.linear.x>SAT) msg.twist.linear.x=SAT; else if (msg.twist.linear.x<-SAT) msg.twist.linear.x=-SAT;
    if (msg.twist.linear.y>SAT) msg.twist.linear.y=SAT; else if (msg.twist.linear.y<-SAT) msg.twist.linear.y=-SAT;
    if (msg.twist.linear.z>SAT) msg.twist.linear.z=SAT; else if (msg.twist.linear.z<-SAT) msg.twist.linear.z=-SAT;

    pub.publish(msg);
  }

}

