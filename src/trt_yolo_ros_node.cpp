#include <trt_yolo_ros/trt_yolo_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trt_yolo_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TrtYoloRos trt_yolo_ros(nh,pnh);
    ros::spin();
    return 0;
}
