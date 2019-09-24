#include <trt_yolo_ros/trt_yolo_ros.h>

TrtYoloRos::TrtYoloRos(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("config_path", config_path_, "/home/nvidia/catkin_ws/src/trt_yolo_ros/config/yolov3-tiny-save.txt");

    char *config_arg[2];
    config_arg[0] = "./apps/trt_yolo_ros/build/trt_yolo_ros";
    //config_arg[1] = const_cast<char *>(("--flagfile=" + config_path_).c_str());
    config_arg[1] = "--flagfile=/home/nvidia/catkin_ws/src/trt_yolo_ros/config/yolov3-tiny-save.txt";

    yoloConfigParserInit(2, config_arg);
    batchSize = getBatchSize();
    yoloInfo = getYoloNetworkInfo();
    yoloInferParams = getYoloInferParams();
    precision = getPrecision();

    inferNet = std::unique_ptr<Yolo>{new YoloV3(batchSize, yoloInfo, yoloInferParams)};

    image_sub_ = nh_.subscribe("/usb_cam/image_raw", 10, &TrtYoloRos::ImageCallback_, this);
    boost::thread publish_thread(boost::bind(&TrtYoloRos::PublishDetectImage_, this));
}

TrtYoloRos::~TrtYoloRos()
{

}

void TrtYoloRos::ImageCallback_(const sensor_msgs::Image::ConstPtr msg)
{
    try 
    {
	img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e) 
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double inferElapsed = 0;

    cv::Mat trtInput = cv::dnn::blobFromImage(img, 1.0f, cv::Size(inferNet->getInputH(), inferNet->getInputW()), cv::Scalar(0.0, 0.0, 0.0), false);

    struct timeval inferStart, inferEnd;
    gettimeofday(&inferStart, NULL);
    inferNet->doInference(trtInput.data, 1);
    gettimeofday(&inferEnd, NULL);
    inferElapsed += ((inferEnd.tv_sec - inferStart.tv_sec)
	         + (inferEnd.tv_usec - inferStart.tv_usec) / 1000000.0) * 1000;

    auto binfo = inferNet->decodeDetections(0, img.rows, img.cols);
    auto remaining = nmsAllClasses(inferNet->getNMSThresh(), binfo, inferNet->getNumClasses());

    for (auto b : remaining)
    {
	printPredictions(b, inferNet->getClassName(b.label));
    }

    std::cout << std::endl
	<< "Network Type : " << inferNet->getNetworkType() << " Precision : " << precision
	<< " Batch Size : " << batchSize
	<< " Inference time per image : " << inferElapsed << " ms"
	<< std::endl;
}

void TrtYoloRos::PublishDetectImage_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        
        //detect_pub_.publish(cmd_vel);

        loop_rate.sleep();
    }
    return;
}
