#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include "ESMTracking.h"
#include "VirtualImage.h"

#include "std_msgs/Float32MultiArray.h"
#include <std_srvs/Empty.h>
#include "uwsimbenchmarks/GTpublish.h"

//Waits for service call to calculate esm tracker initial parameters
class InitService{
 public:
  double init[5];
  int newInit;

  bool initCallback(uwsimbenchmarks::GTpublish::Request  &req, uwsimbenchmarks::GTpublish::Response &res){
    std::vector<double> estimated;

    int i=0;
    estimated.resize(req.groundTruth.size());
    for(std::vector<float>::const_iterator it = req.groundTruth.begin(); it != req.groundTruth.end(); ++it)
      estimated[i++]=*it;


     init[0]=estimated[0];
     init[1]=estimated[1];
     init[3]=sqrt((estimated[4]-estimated[2])*(estimated[4]-estimated[2])+(estimated[5]-estimated[3])*(estimated[5]-estimated[3]));
     init[2]=sqrt((estimated[2]-estimated[0])*(estimated[2]-estimated[0])+(estimated[3]-estimated[1])*(estimated[3]-estimated[1]));
     init[4]=-atan2(estimated[4]-estimated[2],estimated[5]-estimated[3]);
     newInit=1;

    return true;
  }

  InitService(){
    newInit=0;
  }
};


int main(int argc, char **argv){

  std::string image_topic, image_info_topic;
  std::string corners_topic, centroid_topic;
  int reinit;
  std::string autoInitService, startService;

  ros::init(argc, argv, "esm_benchmarkTracking");
  ros::NodeHandle nh;

  //Find parameters or set defaults.
  nh.param("camera", image_topic, (std::string)"uwsim/camera1");
  nh.param("camerainfo", image_info_topic, (std::string)"uwsim/camera1_info");
  nh.param("corners", corners_topic, (std::string)"cornersTopic");
  nh.param("centroid", centroid_topic, (std::string)"centroidTopic");
  nh.param("reinit", reinit, 0);
  nh.param("autoinitservice", autoInitService, (std::string)"autoInit");
  nh.param("startservice", startService, (std::string)"startBench");

  vpImage<vpRGBa> Ic; // Color image

  // Declare a framegrabber able to read ROS images
  VirtualImage g(nh,image_topic,image_info_topic);
  while (!g.ready() && ros::ok()) {ros::spinOnce();}

  // Open the framegrabber by loading the first image of the sequence
  g.open(Ic) ;
  g.acquire(Ic);
  vpDisplayX window(Ic);
  vpDisplay::display(Ic);
  vpDisplay::flush(Ic);
  
  //Create tracker
  ESMTracking * esm;

  //initialize structures to publish centroid and corners
  std::vector<float> corners;
  vpImagePoint centroid;
  ros::Publisher pubcorners = nh.advertise<std_msgs::Float32MultiArray>(corners_topic, 100);
  ros::Publisher pubcentroid = nh.advertise<std_msgs::Float32MultiArray>(centroid_topic, 100);
  std_msgs::Float32MultiArray array;

  //Start autoinit service call start benchmark service and wait to init service.
  std_srvs::Empty::Request request, response;
  //ros::ServiceServer service = nh.advertiseService( autoInitService, &callback2);
  InitService initService;
  ros::ServiceServer serviceServer = nh.advertiseService( autoInitService, &InitService::initCallback,&initService);
  ros::service::call(startService,request, response);

  ros::spinOnce();
  while(!initService.newInit){
    ros::spinOnce();
  }
  initService.newInit=0;

  //Wait until new image arrive
  g.restartReady();
  while (!g.ready() && ros::ok()) {ros::spinOnce();}
  g.acquire(Ic);
  //Start tracker
  esm= new ESMTracking(&Ic,initService.init[0],initService.init[1],initService.init[2],initService.init[3],initService.init[4]);


  while(ros::ok())
  {
    ros::spinOnce();

    //If reinit is set restart tracker when new initial parameters are ready
    if(reinit && initService.newInit){
      free(esm);
      //Wait until new image arrive
      g.restartReady();
      while (!g.ready() && ros::ok()) {ros::spinOnce();}
      g.acquire(Ic);
      esm= new ESMTracking(&Ic,initService.init[0],initService.init[1],initService.init[2],initService.init[3],initService.init[4]);
      initService.newInit=0;
    }
    
    //acquire image parameters
    g.acquire(Ic);
    esm->perceive();

    vpDisplay::display(Ic);
    esm->draw(Ic);

    //get corners and centroid from tracking algorithm
    corners=esm->getCorners();
    centroid=esm->getCentroid();

    //publish centroid
    array.data.clear();
    array.data.push_back(centroid.get_u());
    array.data.push_back(centroid.get_v());  

    pubcentroid.publish(array);

    //publish corners
    array.data.clear();
    for(unsigned int i=0;i<corners.size();i++){
      array.data.push_back(corners[i]);
    }
    pubcorners.publish(array);

    vpDisplay::flush(Ic);
  }

}

