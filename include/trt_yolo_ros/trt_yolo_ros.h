#ifndef TRT_YOLO_ROS_TRT_YOLO_ROS_H_INCLUDED
#define TRT_YOLO_ROS_TRT_YOLO_ROS_H_INCLUDED

#include "trt_utils.h"
#include "yolo_config_parser.h"
#include "yolov3.h"

#include <fstream>
#include <string>
#include <sys/time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class TrtYoloRos
{
public:
    TrtYoloRos(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~TrtYoloRos();
private:
    void PublishDetectImage_(void);
    void ImageCallback_(const sensor_msgs::Image::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string config_path_;
    std::string image_topic_;

    //ros::Publisher detect_pub_;
    ros::Subscriber image_sub_;

    //config parameters
    uint batchSize;
    NetworkInfo yoloInfo;
    InferParams yoloInferParams;
    std::string precision;

    cv::Mat img;
    std::unique_ptr<Yolo> inferNet{nullptr};
};

#endif  //TRT_YOLO_ROS_TRT_YOLO_ROS_H_INCLUDED

