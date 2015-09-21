#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"


//Array topic callback to save last info about corners and centroid of target
class ArrayCallback{
  public:
    std::vector<double> data;
    int ready;
    
    ArrayCallback(){ready=0;}
    void callback(const std_msgs::Float32MultiArray& msg){
     int i=0;
     data.resize(msg.data.size());
     for(std::vector<float>::const_iterator it = msg.data.begin(); it != msg.data.end(); ++it)
       data[i++]=*it;
     ready=1;
    }
};

//Camera info callback to get size of the camera
class InfoCallback{
  public:
    double sizex,sizey;
    double fx,fy;
    InfoCallback(){sizex=0;}
    void callback(const sensor_msgs::CameraInfo& msg) {
      sizex=msg.width;
      sizey=msg.height;
      fx=msg.K[0];
      fy=msg.K[4];
       
    }
};

int main(int argc, char **argv){

  std::string corners_topic, centroid_topic, cam_info,pose_topic;
  double length,width; //We assume length>=width
  ArrayCallback corners,centroid;
  InfoCallback infocb;

  ros::init(argc, argv, "visualStationKeeping");
  ros::NodeHandle nh;

  //Find parameters or set defaults.
  nh.param("corners", corners_topic, (std::string)"cornersTopic");
  nh.param("centroid", centroid_topic, (std::string)"centroidTopic");
  nh.param("cam_info", cam_info, (std::string)"uwsim/camera1_info");
  nh.param("position_pub", pose_topic, (std::string)"gotopose");
  nh.param("objectlength", length, 0.42	);
  nh.param("objectwidth", width, 0.13);

  //Init subscribers
  ros::Subscriber sub_corners = nh.subscribe(corners_topic, 1000, &ArrayCallback::callback,&corners);
  ros::Subscriber sub_centroid = nh.subscribe(centroid_topic, 1000, &ArrayCallback::callback,&centroid);
  ros::Subscriber sub_caminfo = nh.subscribe(cam_info, 1000, &InfoCallback::callback,&infocb);

  while (!corners.ready && ros::ok()) {ros::spinOnce();}

  //Measure pixel-distance relationship
  //As in example benchmark distance to ground does not change, there's no need to estimate it or measure this again.

  //Get edges length
  double d1=sqrt((corners.data[0]-corners.data[2])*(corners.data[0]-corners.data[2])+(corners.data[1]-corners.data[3])*(corners.data[1]-corners.data[3]));
  double d2=sqrt((corners.data[2]-corners.data[4])*(corners.data[2]-corners.data[4])+(corners.data[3]-corners.data[5])*(corners.data[3]-corners.data[5]));
  //std::cout<<d1<<" "<<d2<<std::endl;

  //get box angle wrt xy axis
  double angx1=atan((corners.data[1]-corners.data[3])/(corners.data[0]-corners.data[2]));
  double angy1=atan((corners.data[0]-corners.data[2])/(corners.data[1]-corners.data[3]));

  double angx2=atan((corners.data[3]-corners.data[5])/(corners.data[2]-corners.data[4]));
  double angy2=atan((corners.data[2]-corners.data[4])/(corners.data[3]-corners.data[5]));

  //get  xy distances
  double xdist1=cos(angx1)*d1;
  double ydist1=cos(angy1)*d1;

  double xdist2=cos(angx2)*d2;
  double ydist2=cos(angy2)*d2;

  //check if we need to invert length width
  if(d1<d2){
    double aux=width;
    width=length;
    length=aux;
  }

  //use x and y longest distance 
  double resx,resy;
  if(xdist1 > xdist2){
    resx=(length*cos(angx1))/xdist1;
    resy=(width*cos(angy2))/ydist2;
  }  
  else{
    resx=(width*cos(angx2))/xdist2;
    resy=(length*cos(angy1))/ydist1;
  }

  //Calculate z reference (at we should stay)	
  double zref=(length*infocb.fx/d1 + width*infocb.fx/d2)/2;

  //Wait for camera info
  while (!infocb.sizex && ros::ok()) {ros::spinOnce();}

  //Start publisher
  ros::Publisher pubpose = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 100);

  ros::Rate loop_rate(50);
  while(ros::ok()){
    ros::spinOnce();
	
    //As soon as we have centroid information estimate distance to target and publish
    if(centroid.ready){
      //Get z
      d1=sqrt((corners.data[0]-corners.data[2])*(corners.data[0]-corners.data[2])+(corners.data[1]-corners.data[3])*(corners.data[1]-corners.data[3]));
      d2=sqrt((corners.data[2]-corners.data[4])*(corners.data[2]-corners.data[4])+(corners.data[3]-corners.data[5])*(corners.data[3]-corners.data[5]));

      double z=(length*infocb.fx/d1 + width*infocb.fx/d2)/2;

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x=((infocb.sizey/2)-centroid.data[1])*resy;  //In current example camera is placed like this x->-y y->x
      pose.pose.position.y=-((infocb.sizex/2)-centroid.data[0])*resx;
      pose.pose.position.z=z-zref;

      pose.pose.orientation.x=0;
      pose.pose.orientation.y=0;
      pose.pose.orientation.z=0;
      pose.pose.orientation.w=1;
      pubpose.publish(pose); 
      centroid.ready=0;

    }


    loop_rate.sleep();

  }

}

