#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp_bridge/image.h>

//Warp
#include <visp/vpTemplateTrackerWarpAffine.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTemplateTrackerWarpHomographySL3.h>
#include <visp/vpTemplateTrackerWarpSRT.h>
#include <visp/vpTemplateTrackerWarpTranslation.h>

//Template trackers
#include <visp/vpTemplateTrackerSSD.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>

#include <visp/vpImageConvert.h>

#include <std_srvs/Empty.h>
#include "VirtualImage.h"
#include "uwsimbenchmarks/GTpublish.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/String.h>

//Waits for service call to calculate esm tracker initial parameters
class InitService{
 public:
  double init[8];
  int newInit;

  bool initCallback(uwsimbenchmarks::GTpublish::Request  &req, uwsimbenchmarks::GTpublish::Response &res){
    std::vector<double> estimated;

    int i=0;
    estimated.resize(req.groundTruth.size());
    for(std::vector<float>::const_iterator it = req.groundTruth.begin(); it != req.groundTruth.end(); ++it)
      estimated[i++]=*it;


     init[0]=estimated[0];
     init[1]=estimated[1];
     init[2]=estimated[2];
     init[3]=estimated[3];
     init[4]=estimated[4];
     init[5]=estimated[5];
     init[6]=estimated[6];
     init[7]=estimated[7];
     newInit=1;

    return true;
  }

  InitService(){
    newInit=0;
  }
};

//Reads benchmark information topic (checks when first scene updater changes scene)
class BenchmarkInfoCallback{
  public:
    int newIteration;

    BenchmarkInfoCallback(){
      newIteration=0;
    }

    void callback(const std_msgs::String& msg) {
      if(msg.data=="Updated scene 1")
	newIteration=1;
    }
};

//Creates desired warp
vpTemplateTrackerWarp * chooseWarp(std::string warp_type){
  if(warp_type=="WARP_AFFINE")
    return new vpTemplateTrackerWarpAffine;
  else if(warp_type=="WARP_HOMOGRAPHY")
    return new vpTemplateTrackerWarpHomography;
  else if(warp_type=="WARP_HOMOGRAPHY_SL3")
    return new vpTemplateTrackerWarpHomographySL3;
  else if(warp_type=="WARP_SRT")
    return new vpTemplateTrackerWarpSRT;
  else if(warp_type=="WARP_TRANSLATION")
    return new vpTemplateTrackerWarpTranslation;
  else{
    std::cerr<<"warp type not found"<<std::endl;
    exit(0);
  }
}

//Creates desired tracker using a wrap
vpTemplateTracker * chooseTracker(std::string tracker_type, vpTemplateTrackerWarp * warp){
  if(tracker_type=="TRACKER_SSD_ESM")
    return new vpTemplateTrackerSSDESM(warp);
  else if(tracker_type=="TRACKER_SSD_FORWARD_ADDITIONAL")
    return new vpTemplateTrackerSSDForwardAdditional(warp);
  else if(tracker_type=="TRACKER_SSD_FORWARD_COMPOSITIONAL")
    return new vpTemplateTrackerSSDForwardCompositional(warp);
  else if(tracker_type=="TRACKER_SSD_INVERSE_COMPOSITIONAL")
    return new vpTemplateTrackerSSDInverseCompositional(warp);
  else if(tracker_type=="TRACKER_ZNCC_FORWARD_ADDITIONAL")
    return new vpTemplateTrackerZNCCForwardAdditional(warp);
  else if(tracker_type=="TRACKER_ZNCC_INVERSE_COMPOSITIONAL")
    return new vpTemplateTrackerZNCCInverseCompositional(warp);
  else{
    std::cerr<<"tracker type not found"<<std::endl;
    exit(0);
  }
}
 
int main(int argc, char **argv)
{
  std::string image_topic, image_info_topic;
  std::string corners_topic, centroid_topic;
  int reinit,changeTrackerOnUpdate;
  std::string autoInitService, startService;
  std::string tracker_type, warp_type;
  std::string benchinfo_topic;
  std::string publishTracker;
  BenchmarkInfoCallback benchInfo;

  int tracker_number=0, warp_number=0;
  ros::init(argc, argv, "vispTracking");
  ros::NodeHandle nh;
 
 #ifdef VISP_HAVE_X11
  vpDisplayX display;
 #else
  vpDisplayGDI display;
 #endif

  vpImage<unsigned char> I; // Grayscale image
  vpImage<vpRGBa> Ic; // Color image

  //Find parameters or set defaults.
  nh.param("camera", image_topic, (std::string)"uwsim/camera1");
  nh.param("camerainfo", image_info_topic, (std::string)"uwsim/camera1_info");
  nh.param("corners", corners_topic, (std::string)"cornersTopic");
  nh.param("centroid", centroid_topic, (std::string)"centroidTopic");
  nh.param("reinit", reinit, 0);
  nh.param("autoinitservice", autoInitService, (std::string)"autoInit");
  nh.param("startservice", startService, (std::string)"startBench");
  nh.param("tracker_type", tracker_type, (std::string)"TRACKER_ZNCC_INVERSE_COMPOSITIONAL");
  nh.param("warp_type", warp_type, (std::string)"WARP_SRT");
  nh.param("benchinfo", benchinfo_topic, (std::string)"BenchmarkInfo");
  nh.param("changeTrackerOnUpdate", changeTrackerOnUpdate, 0);
  nh.param("publishTracker", publishTracker, (std::string)"");


  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_tracker;
  if(publishTracker!="")
    pub_tracker = it.advertise(publishTracker, 1);

  //Types of tracker:
  std::vector<std::string> tracker_types;
  tracker_types.push_back("TRACKER_SSD_FORWARD_ADDITIONAL");
  tracker_types.push_back("TRACKER_SSD_FORWARD_COMPOSITIONAL");
  tracker_types.push_back("TRACKER_SSD_INVERSE_COMPOSITIONAL");
  tracker_types.push_back("TRACKER_ZNCC_FORWARD_ADDITIONAL");
  tracker_types.push_back("TRACKER_ZNCC_INVERSE_COMPOSITIONAL");
  tracker_types.push_back("TRACKER_SSD_ESM");

  //warp types
  std::vector<std::string> warp_types;
  warp_types.push_back("WARP_AFFINE");
  warp_types.push_back("WARP_HOMOGRAPHY");
  warp_types.push_back("WARP_HOMOGRAPHY_SL3");
  warp_types.push_back("WARP_SRT");
  warp_types.push_back("WARP_TRANSLATION");

  // Declare a framegrabber able to read ROS images
  VirtualImage g(nh,image_topic,image_info_topic);
  while (!g.ready() && ros::ok()) {ros::spinOnce();}

  // Open the framegrabber by loading the first image of the sequence
  g.open(Ic) ;
  g.acquire(Ic);
  vpImageConvert::convert(Ic,I);
  vpDisplayX window(Ic);
  vpDisplay::display(Ic);
  vpDisplay::flush(Ic);

  //initialize structures to publish centroid and corners
  std::vector<float> corners;
  vpImagePoint centroid;
  ros::Publisher pubcorners = nh.advertise<std_msgs::Float32MultiArray>(corners_topic, 100);
  ros::Publisher pubcentroid = nh.advertise<std_msgs::Float32MultiArray>(centroid_topic, 100);
  std_msgs::Float32MultiArray array;

  //Start autoinit service call start benchmark service
  std_srvs::Empty::Request request, response;
  InitService initService;
  ros::ServiceServer serviceServer = nh.advertiseService( autoInitService, &InitService::initCallback,&initService);
  ros::service::call(startService,request, response);
  ros::Subscriber sub_benchInfo = nh.subscribe(benchinfo_topic, 1000, &BenchmarkInfoCallback::callback,&benchInfo);

  //If change tracker on update start with first tracker and warp
  if(changeTrackerOnUpdate){
    warp_type=warp_types[warp_number];
    tracker_type=tracker_types[tracker_number];
  }
  //create warp
  vpTemplateTrackerWarp *warp = chooseWarp(warp_type);

  //Create template tracker
  vpTemplateTracker *tracker = chooseTracker(tracker_type,warp);

  //TODO allow change settings
  tracker->setSampling(2,2);
  tracker->setLambda(0.001);
  tracker->setIterationMax(200);
  tracker->setPyramidal(2, 1);

  //wait to init service.
  ros::spinOnce();
  while(!initService.newInit){
    ros::spinOnce();
  }
  initService.newInit=0;

  //Wait until new image arrive
  g.restartReady();
  while (!g.ready() && ros::ok()) {ros::spinOnce();}
  g.acquire(Ic);
  vpImageConvert::convert(Ic,I);

  //Init tracker 
  std::vector< vpImagePoint > vector;
  vector.push_back(vpImagePoint(initService.init[1],initService.init[0]));
  vector.push_back(vpImagePoint(initService.init[3],initService.init[2]));
  vector.push_back(vpImagePoint(initService.init[5],initService.init[4]));
  vector.push_back(vpImagePoint(initService.init[5],initService.init[4]));
  vector.push_back(vpImagePoint(initService.init[7],initService.init[6]));
  vector.push_back(vpImagePoint(initService.init[1],initService.init[0]));

  //tracker->initClick(I);
  tracker->initFromPoints (I, vector);

  // Instantiate and get the reference zone
  vpTemplateTrackerZone zone_ref = tracker->getZoneRef();
  // Instantiate a warped zone
  vpTemplateTrackerZone zone_warped;

 std::cout<<"Now tracker is "<<tracker_type<<" and warp "<<warp_type<<std::endl;
 while(1){
    //Acquite image and convert it to bw (tracker needs it)
    ros::spinOnce();
    g.acquire(Ic);
    vpImageConvert::convert(Ic,I);
    vpDisplay::display(Ic);

  //Check if change tracker on update is needed
  if(changeTrackerOnUpdate && benchInfo.newIteration){
    warp_number++;
    if(warp_number >= warp_types.size()){
      warp_number=0;
      tracker_number++;
    }
    warp_type=warp_types[warp_number];
    tracker_type=tracker_types[tracker_number];

    std::cout<<"Now tracker is "<<tracker_type<<" and warp "<<warp_type<<std::endl;
    free(warp);
    free(tracker);
    warp = chooseWarp(warp_type);
    tracker = chooseTracker(tracker_type,warp);

    //Wait until new image arrive
    g.restartReady();
    while (!g.ready() && ros::ok()) {ros::spinOnce();}
    g.acquire(Ic);
    vpImageConvert::convert(Ic,I);

    vpDisplay::display(Ic);
   
    //reinit tracker with starting information
    tracker->initFromPoints (I, vector);
    vpTemplateTrackerZone zone_ref = tracker->getZoneRef();	

    benchInfo.newIteration=0;
  }

  //Check if reinitialize is needed
  if(reinit && initService.newInit){
    free(warp);
    free(tracker);

    warp = chooseWarp(warp_type);
    tracker = chooseTracker(tracker_type,warp);

    //Wait until new image arrive
    g.restartReady();
    while (!g.ready() && ros::ok()) {ros::spinOnce();}
    g.acquire(Ic);
    vpImageConvert::convert(Ic,I);

    vpDisplay::display(Ic);
    
    //Restart tracker
    vector.clear();
    vector.push_back(vpImagePoint(initService.init[1],initService.init[0]));
    vector.push_back(vpImagePoint(initService.init[3],initService.init[2]));
    vector.push_back(vpImagePoint(initService.init[5],initService.init[4]));
    vector.push_back(vpImagePoint(initService.init[5],initService.init[4]));
    vector.push_back(vpImagePoint(initService.init[7],initService.init[6]));
    vector.push_back(vpImagePoint(initService.init[1],initService.init[0])); 
    tracker->initFromPoints (I, vector);

    vpTemplateTrackerZone zone_ref = tracker->getZoneRef();
	

    initService.newInit=0;
  }

    vpImagePoint centroid; 
    try{ //Check if tracking fails
     tracker->track(I);
     tracker->display(Ic, vpColor::green);

    if (vpDisplay::getClick(Ic, false))
      break;

    // Get the estimated parameters
    vpColVector p = tracker->getp();

    // Update the warped zone given the tracker estimated parameters
    warp->warpZone(zone_ref, p, zone_warped);

    centroid =zone_warped.getCenter();
    }
    catch (const std::exception& ex) {
    centroid=vpImagePoint(1000,1000);//Error value!
    }
    catch (const std::string& ex) {
    }


 
    //publish centroid
    array.data.clear();
    array.data.push_back(centroid.get_u());
    array.data.push_back(centroid.get_v());  

    pubcentroid.publish(array);

    //std::cout<<"CENTROID: "<<centroid.get_u()<<" "<<centroid.get_v()<<std::endl;

    //publish corners
    array.data.clear();
    array.data.push_back(zone_warped.getMinx());
    array.data.push_back(zone_warped.getMiny());
    array.data.push_back(zone_warped.getMaxx());
    array.data.push_back(zone_warped.getMiny());
    array.data.push_back(zone_warped.getMaxx());
    array.data.push_back(zone_warped.getMaxy());
    array.data.push_back(zone_warped.getMinx());
    array.data.push_back(zone_warped.getMaxy());

    pubcorners.publish(array);
    vpDisplay::flush(Ic);
    if(publishTracker!=""){
      vpImage<vpRGBa> Ioverlay;
      vpDisplay::getImage(Ic, Ioverlay) ;
      pub_tracker.publish(visp_bridge::toSensorMsgsImage(Ioverlay));
    }

    vpTime::wait(40);
 }
}

