#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "underwater_sensor_msgs/DVL.h"
#define SAT	5

//Twist callback to get velocity reference
class TwistCallback{
  public:
    double linear[3];
    double angular[3];

    TwistCallback(){
	linear[0]=linear[1]=linear[2]=0;
	angular[0]=angular[1]=angular[2]=0;
    }
 
    void callback(const geometry_msgs::TwistStamped& msg) {
      linear[0]=msg.twist.linear.x;
      linear[1]=msg.twist.linear.y;
      linear[2]=msg.twist.linear.z;

      angular[0]=msg.twist.angular.x;
      angular[1]=msg.twist.angular.y;
      angular[2]=msg.twist.angular.z;
    }
};

//DVL callback to get vehicle's velocity
class DVLCallback{
  public:
    double linear[3];

    DVLCallback(){
	linear[0]=linear[1]=linear[2]=0;
    }
 
    void callback(const underwater_sensor_msgs::DVL& msg) {
      linear[0]=msg.bi_x_axis;
      linear[1]=msg.bi_y_axis;
      linear[2]=msg.bi_z_axis;
    }
};

int main(int argc, char **argv){
  std::string twist_topic, thrusters_topic, dvl_topic;
  TwistCallback twist;
  DVLCallback dvl;

  ros::init(argc, argv, "vehicleThrusterAllocator");
  ros::NodeHandle nh;
 
  nh.param("twist", twist_topic, (std::string)"/dataNavigator");
  nh.param("thrusters", thrusters_topic, (std::string)"/g500/thrusters_input");
  nh.param("dvl", dvl_topic, (std::string)"/g500/dvl");

  ros::Subscriber sub_twist = nh.subscribe(twist_topic, 1000, &TwistCallback::callback,&twist);
  ros::Subscriber sub_dvl = nh.subscribe(dvl_topic, 1000, &DVLCallback::callback,&dvl);
  ros::Publisher pub=nh.advertise<std_msgs::Float64MultiArray>(thrusters_topic, 1);

  ros::Rate loop_rate(50);
  double thrust_req[5];
  sleep(10);
  while(ros::ok())
  {
    memset(thrust_req, 0, sizeof(thrust_req)); //clear array
    //std::cout<<twist.linear[0]-dvl.linear[0]<<" "<<twist.linear[1]-dvl.linear[1]<<" "<<twist.linear[2]-dvl.linear[2]<<std::endl;
    //std::cout<<"DESIRED: "<<twist.linear[2]<<" REAL:"<<dvl.linear[2]<<" APLICADA: "<<twist.linear[2]-dvl.linear[2]<<std::endl;

    thrust_req[0]=(-twist.linear[0]-dvl.linear[0])*5;//-1.142857143*twist.linear[0];
    thrust_req[1]=(-twist.linear[0]-dvl.linear[0])*5;//-1.142857143*twist.linear[0];
    thrust_req[2]=-(twist.linear[2]-dvl.linear[2])*5; //arriba y abajo
    thrust_req[3]=-(twist.linear[2]-dvl.linear[2])*5;
    thrust_req[4]=(twist.linear[1]+dvl.linear[1])*6;//1.6*twist.linear[1];

    //std::cout<<thrust_req[0]<<" "<<thrust_req[1]<<" "<<thrust_req[2]<<" "<<thrust_req[3]<<" "<<thrust_req[4]<<std::endl;

    //Send message to Simulator
    std_msgs::Float64MultiArray msg;

    msg.data.push_back(thrust_req[0]);
    msg.data.push_back(thrust_req[1]);
    msg.data.push_back(thrust_req[2]);
    msg.data.push_back(thrust_req[3]);
    msg.data.push_back(thrust_req[4]);
   
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

}

