#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
ros::Publisher pub;
int main(int argc,char** argv)
{
	ros::init(argc, argv, "getimage");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    std::string IpLastSegment = "152";
    int cam = 2;
    if (argc>=2)
    {
        cam = std::atoi(argv[1]);
    }
    std::string udpstrPrevData = "udpsrc address=192.168.123."+ IpLastSegment + " port=";
    std::array<int,5> udpPORT = std::array<int, 5>{9201, 9202, 9203, 9204, 9205};
    std::string udpstrBehindData = " ! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    std::string udpSendIntegratedPipe = udpstrPrevData +  std::to_string(udpPORT[cam-1]) + udpstrBehindData;
    std::cout<<"udpSendIntegratedPipe:"<<udpSendIntegratedPipe<<std::endl;
    cv::VideoCapture cap(udpSendIntegratedPipe);
    if(!cap.isOpened())
    {
		ROS_INFO("NO IMAGE INPUT1");
		return 0;
    } 
    cv::Mat image;
    while(1)
    {
        cap >> image;
        if(image.empty())
        {
			ROS_INFO("NO IMAGE INPUT");
			break;
        }
        else
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_transport::ImageTransport it(nh);
            //topic name is /camera_front/image_color,the publish message queue size is 1.
            image_transport::Publisher pub = it.advertise("image", 10);
            pub.publish(msg);
            cv::imshow("image",image);
            cv::waitKey(0);
        }
    cap.release();//释放资源
    while(nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    }
}