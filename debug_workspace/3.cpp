#include <ros/ros.h>  
#include <sensor_msgs/Image.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/opencv.hpp>  
  
int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "getimage");  
    ros::NodeHandle nh;  
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("image", 10);  
  
    std::string IpLastSegment = "152";  
    int cam = 2;  
    if (argc >= 2) {  
        cam = std::atoi(argv[1]);  
    }  
  
    std::string udpstrPrevData = "udpsrc address=192.168.123."+ IpLastSegment + " port=";
    std::array<int,5> udpPORT = std::array<int, 5>{9201, 9202, 9203, 9204, 9205};
    std::string udpstrBehindData = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    std::string udpSendIntegratedPipe = udpstrPrevData +  std::to_string(udpPORT[cam-1]) + udpstrBehindData;
    std::cout<<"udpSendIntegratedPipe:"<<udpSendIntegratedPipe<<std::endl;
    cv::VideoCapture cap(udpSendIntegratedPipe);
    if (!cap.isOpened()) {  
        ROS_ERROR("Failed to open video capture");  
        return 1;  
    }  
  
    cv::Mat image;  
    ros::Rate loop_rate(10);  // Adjust the rate as needed  
  
    while (ros::ok() && cap.isOpened()) {  
        cap >> image;  
        if (image.empty()) {  
            ROS_WARN("No image captured");  
            continue;  
        }  
  
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();  
        pub.publish(msg);  
  
        // Optionally show the image  
        // cv::imshow("image", image);  
        // cv::waitKey(1);  // Reduce the wait time to avoid blocking  
  
        ros::spinOnce();  
        loop_rate.sleep();  
    }  
  
    cap.release();  
    ROS_INFO("Shutting down image capture node");  
    return 0;  
}