#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>


#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>
#include <visp3/io/vpImageIo.h>

#include <geometry_msgs/Pose.h>

#include <vector>

class QRDetector {

	/* Private variables */

	vpImage<unsigned char> I; // Create a gray level image container
    	//vpImage<vpRGBa> I; // Create a color image container

	bool detected;

   	//! [Construction]
   	vpROSGrabber g; // Create a grabber based on ROS
	//! [Construction]

	// Declare the QR detector
    	vpDetectorBase *detector = NULL;

	//! [Pose computation]
	vpHomogeneousMatrix cMo;
	// 3D model of the QRcode
	std::vector<vpPoint> point;
        //! [Pose computation]


	/* Private functions */
	void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo);
	void getPose(vpHomogeneousMatrix& cMo, geometry_msgs::Pose& qr_pose);
	

    public:

	/* Constructor */
	QRDetector();

	/* Destructor */
	~QRDetector();

	/* QR detection */
	bool detect();

	/* Auxiliar */
	geometry_msgs::Pose getPose();
	bool isQR();

};
