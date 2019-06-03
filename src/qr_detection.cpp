//! EDITED FROM \example tutorial-ros-grabber.cpp
#include <youbot_pickup/qr_detection.h>

/* Constructor */
QRDetector::QRDetector()
{
    
  try{

    //! [Setting camera topic]
    g.setImageTopic("/camera/rgb/image_raw");
    //! [Setting camera topic]
    //! [Setting camera info]
    g.setCameraInfoTopic("/camera/rgb/camera_info");
    g.setRectify(true);
    //! [Setting camera info]
  
    detected = false;
    
    
    // 3D model of the QRcode (6cm each side, change this for other metrics)
    point.push_back(vpPoint(-0.03, -0.03, 0)); // QCcode point 0 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.03, -0.03, 0));  // QCcode point 1 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.03, 0.03, 0));   // QCcode point 2 3D coordinates in plane Z=0
    point.push_back(vpPoint(-0.03, 0.03, 0));  // QCcode point 3 3D coordinates in plane Z=0

  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
  
}

/* Destructor */
QRDetector::~QRDetector(){

  delete detector;
}

void QRDetector::computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }
  if (init == true) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

bool QRDetector::detect(){

  try {
    //! [Opening]
    g.open(I);
    //! [Opening]
    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif


#if defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
#else
    std::cout << "You don't have ZBar installed. Please install: https://visp.inria.fr/3rd_zbar/" << std::endl;
#endif

    bool init = true;
    // Camera parameters
    vpCameraParameters cam(g.getWidth(), g.getHeight(), I.getWidth() / 2, I.getHeight() / 2);
    ROS_INFO("Camera parameters settled.");

    while(!detected && ros::ok()) {
      //! [Acquisition]
      g.acquire(I);
      //! [Acquisition]
      vpDisplay::display(I);

      vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);

      
      // QR detection code
      bool status = detector->detect(I);
      std::ostringstream legend;
      legend << detector->getNbObjects() << " bar code detected";
      //ROS_INFO_STREAM(legend.str());
      vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);
      if (status) {
	detected = true;
        for (size_t i = 0; i < detector->getNbObjects(); i++) {
          std::vector<vpImagePoint> p = detector->getPolygon(i);
          vpRect bbox = detector->getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green);
          vpDisplay::displayText(I, (int)(bbox.getTop() - 10), (int)bbox.getLeft(), "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
          for (size_t j = 0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
          }
	  ROS_INFO("Going to compute pose...");
	  computePose(point, p, cam, init, cMo); // resulting pose is available in cMo global var
          ROS_INFO_STREAM(std::endl << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                    << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t());
          vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        }
	vpDisplay::flush(I);
      }
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    detected = false;
    return false;
  }
  return true;
}

void QRDetector::getPose(vpHomogeneousMatrix& cMo, geometry_msgs::Pose& qr_pose){

  try{

    if(detected){    
    
        std::vector<double> t;
 	vpRowVector vpT;
  	vpT =  cMo.getTranslationVector().t();
  	t.push_back(vpT[0]);
  	t.push_back(vpT[1]);
  	t.push_back(vpT[2]);

	vpQuaternionVector vpQ = vpQuaternionVector(cMo.getRotationMatrix());

	qr_pose.position.x = t[0];
  	qr_pose.position.y = t[1];
  	qr_pose.position.z = 0;

  	qr_pose.orientation.x = 0;
  	qr_pose.orientation.y = 0;
  	qr_pose.orientation.z = vpQ.z();
  	qr_pose.orientation.w = vpQ.w();

	detected = false;

    } else {
	ROS_ERROR("QR not detected.");
    }
      
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  } 
 
}

bool QRDetector::isQR(){

  return detected;
}
 
/* Auxiliar functions (public) */
geometry_msgs::Pose QRDetector::getPose(){
  geometry_msgs::Pose qr_pose;
  getPose(cMo, qr_pose);
  return qr_pose;
}
