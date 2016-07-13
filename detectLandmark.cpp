#include "landmark_detection.h"


landmarkDetection::landmarkDetection()
{
    //class constructor
    landmarksDetected = false;

}

void landmarkDetection::imageReceivedCallback(const sensor_msgs::ImageConstPtr& img)
{
	cv_bridge::CvImagePtr cv_ptr;
    ros::Time timestamp;

    try
    {
    	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
   
	Mat image = cv_ptr->image;
    timestamp = img->header.stamp;


    Mat imgH;
    image.convertTo(imgH, -1, 0.5, 0);

    //imshow("Contrast Reduced", imgH);
    cv::GaussianBlur(imgH, imgH, Size(9,9), 0, 0);

    int erosion_size = 4;
    Mat erosion_dst;
    Mat erosion_element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
    /// Apply the erosion operation
    erode( imgH, erosion_dst, erosion_element );
    cv::GaussianBlur(erosion_dst, erosion_dst, Size(9,9), 0, 0);

    //imshow( "Erosion Output", erosion_dst );

    Mat hsv;
    cvtColor(erosion_dst,hsv,CV_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 100, 255), lower_red_hue_range);
    inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

    // Combine the above two images
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);


    int dilation_size = 10;
    Mat dilation_dst;
    Mat dilation_element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 4*dilation_size + 1, 4*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( red_hue_image, dilation_dst, dilation_element );
    cv::GaussianBlur(dilation_dst, dilation_dst, Size(9,9), 0, 0);
    imshow( "Dilation Output", dilation_dst );


    cv::threshold(dilation_dst, dilation_dst, 200, 300, CV_THRESH_BINARY);

    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours( dilation_dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    if(contours.size() == 2)
    {
        landmarksDetected = true;
        //cout << "landmarks detected" << endl;
    }

    else
    {
        landmarksDetected = false;
        //cout << "landmarks not detected" << endl;
    }

    map.insert(std::pair<ros::Time,bool>(timestamp,landmarksDetected));

    imshow( "Contours", dilation_dst );
    imshow("Original Image", image);
    cv::waitKey(3);

}

bool landmarkDetection::getDetectionStatus(ros::Time timestamp)
{
    return landmarkDetection::map.find(timestamp)->second;
}



