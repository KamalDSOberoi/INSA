


/* The process to detect the landmarks needs to be more robust with varying light conditions;
 * only then the idea to optimize the map when the loop closue is detected will work.
 * Right now, the problem is with the light coming from the windows and the light reflected from the walls. 
 * 
 */



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

    // reject 350 pixels from below; 150 pixels from above
    cv::Rect roi = cv::Rect(0, 150, image.cols, image.rows-350);
    Mat resizedImage(image, roi);
    imshow("resized image", resizedImage);


    Mat resizedImage_gray, threshold_output;
    cvtColor(resizedImage, resizedImage_gray, CV_BGR2GRAY);
    imshow("resizedImage_gray", resizedImage_gray);
    
    threshold(resizedImage_gray, threshold_output, 250, 400, THRESH_BINARY); 
    imshow("threshold_output", threshold_output);                            

    int dilation_size = 5;
    Mat dilation_dst;
    Mat dilation_element = getStructuringElement( MORPH_RECT,
                                       Size( 4*dilation_size + 1, 4*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate( threshold_output, dilation_dst, dilation_element );
    cv::GaussianBlur(dilation_dst, dilation_dst, Size(9,9), 0, 0);
    imshow( "Dilation Output", dilation_dst );

    /*vector<vector<Point> > contour;
    std::vector<Vec4i> hierarch;
    findContours(threshold_output, contour, hierarch, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    //imshow("threshold_output", threshold_output);

    std::cout << "contour.size: " << contour.size() << std::endl;

    vector<RotatedRect> minEllipse(contour.size());
    vector<RotatedRect> minRect(contour.size());

    for( int i = 0; i < contour.size(); i++ )
    {
        minRect[i] = minAreaRect(Mat(contour[i])); 
        if(contour[i].size() > 5 )
        { 
            minEllipse[i] = fitEllipse(Mat(contour[i])); 
        }
    }

    RNG rng(12345);
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contour.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // contour
        drawContours( drawing, contour, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse
        ellipse( drawing, minEllipse[i], color, 2, 8 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    imshow( "drawing", drawing );
    

    /*Mat hsv;
    //cvtColor(erosion_dst,hsv,CV_BGR2HSV);

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
    //imshow( "Dilation Output", dilation_dst );


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

    //imshow( "Contours", dilation_dst );
    //imshow("Image", image);*/


    cv::waitKey(3);

}

bool landmarkDetection::getDetectionStatus(ros::Time timestamp)
{
    return landmarkDetection::map.find(timestamp)->second;
}



