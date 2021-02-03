#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";

// Declare the output variables
 Mat dst, cdst, cdstP;
 
class imgProc {

	ros::NodeHandle nh;
	image_transport::ImageTransport it;    
	image_transport::Subscriber imgSub; 
	image_transport::Publisher imgPub;
	image_transport::Publisher imgHough_pub;
	image_transport::Publisher imgHoughP_pub;

	void publishOpenCVImage(image_transport::Publisher pub, Mat img) {
		cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img);
		sensor_msgs::Image msg;
		imgBridge.toImageMsg(msg);
		pub.publish(msg);
	}
    public:
	imgProc() : it(nh) {
		imgSub = it.subscribe("/usb_cam/image_raw", 1, &imgProc::process, this);
		imgPub = it.advertise("/camera/canny_edge",1);
		imgHough_pub = it.advertise("/camera/hough_lines",1);
		imgHoughP_pub = it.advertise("/camera/hough_probabilistic_lines",1);
        cv::namedWindow(OPENCV_WINDOW);
	}
    ~imgProc()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  
	void process(const sensor_msgs::ImageConstPtr& srcImg) {
		cv_bridge::CvImagePtr cv_ptr;
    		try {
			cv_ptr = cv_bridge::toCvCopy(srcImg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//canny edge detection
        int theshold1 = 50;
		int theshold2 = 100; 
		int kernelSize = 3;

		Mat img;
		Mat gray;
		Mat sliba;
		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY); //keeping grayscale image
		cvtColor(gray, sliba, CV_GRAY2BGR);
		cvtColor(cv_ptr->image, img, CV_BGR2GRAY);
		GaussianBlur(img, img, Size(3, 3), 0, 0);
      	Canny(img, dst, theshold1, theshold2, kernelSize);

		// Copy edges to the images that will display the results in BGR
		cvtColor(dst, cdst, CV_GRAY2BGR);
		cdstP = cdst.clone();


		// Standard Hough Line Transform
   		vector<Vec2f> lines; // will hold the results of the detection
    	HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    	// Draw the lines
    	for( size_t i = 0; i < lines.size(); i++ )
    	{
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    	}



		 // Probabilistic Line Transform
    	vector<Vec4i> linesP; // will hold the results of the detection
    	HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    	// Draw the lines
    	for( size_t i = 0; i < linesP.size(); i++ )
    	{
        Vec4i l = linesP[i];
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    	}



		//Hough Circle Transform
		//medianBlur(gray, gray, 5);
    	/*vector<Vec3f> circles;
    	HoughCircles(dst, circles, HOUGH_GRADIENT, 1,
                 dst.rows/8,  // change this value to detect circles with different distances to each other
                 100, 30, 1, 60 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    	);
    	for( size_t i = 0; i < circles.size(); i++ )
    	{
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle( cdstP, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( cdstP, center, radius, Scalar(255,0,255), 3, LINE_AA);
    	} */

         // Update GUI Window
        //cv::imshow(OPENCV_WINDOW, dst); //canny image (edges detected)
        //cv::waitKey(1);

		//cv::imshow(OPENCV_WINDOW, cdst); //standard Houghe line transform
        //cv::waitKey(2);

	cv::imshow(OPENCV_WINDOW, cdstP); //Probabilistic Houghe line transform
        cv::waitKey(3);

		//publishing images
		publishOpenCVImage(imgPub, dst); 
		//publishOpenCVImage(imgHough_pub, cdst);
		//publishOpenCVImage(imgHoughP_pub, cdstP);

	}
};


int main(int argc, char** argv) {
	ros::init(argc, argv, "img_processing_node");
	imgProc worker;
	ros::spin();

	return 0;
}
