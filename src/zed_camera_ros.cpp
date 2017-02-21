#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <camera_info_manager/camera_info_manager.h>

using namespace cv;

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

void setResolution(int type, cv::VideoCapture* camera_);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher left_pub =
    it.advertiseCamera("/zed/left/image_raw", 1);
  image_transport::CameraPublisher right_pub =
    it.advertiseCamera("/zed/right/image_raw", 1);

  camera_info_manager::CameraInfoManager left_cam_info(nh, "zed/left");
  camera_info_manager::CameraInfoManager right_cam_info(nh, "zed/right");

  left_cam_info.loadCameraInfo(
      "package://zed_camera_ros/params/ZedLeft_360p.yaml");
  right_cam_info.loadCameraInfo(
      "package://zed_camera_ros/params/ZedRight_360p.yaml");
 
  if (!left_cam_info.isCalibrated()) {
    ROS_ERROR("Cannot load left camera calibration");
    return 1; 
  }
  if (!right_cam_info.isCalibrated()) {
    ROS_ERROR("Cannot load right camera calibration");
    return 1;
  }

  sensor_msgs::CameraInfoPtr cil(
      new sensor_msgs::CameraInfo(left_cam_info.getCameraInfo()));
  sensor_msgs::CameraInfoPtr cir(
      new sensor_msgs::CameraInfo(right_cam_info.getCameraInfo()));

  int fps;

  if (ros::param::get("~frame_rate", fps))
  {
    ROS_INFO("Running Camera with Frame Rate at %d fps", fps);
  }
  else
  {
    ROS_ERROR("Failed to get param 'frame_rate', set to default value (30)");
    fps = 30;
  }

  int frameSkip = 0;

  frameSkip = 60 / fps - 1;

  int res_; 

  if (ros::param::get("~resolution", res_))
  {
    switch (res_)
    {
      case 0:
        ROS_INFO("Running Camera with resolution at 640x360");
        break;
      case 1:
        ROS_INFO("Running Camera with resolution at 960x540");
        break;
      case 2:
        ROS_INFO("Running Camera with resolution at 1280x720");
        break;
    }
  }
  else
  {
    ROS_ERROR("Failed to get param 'resolution', set to default value (0)");
    res_ = 0;
  }

  ros::NodeHandle nh_priv("~");
  int camera_number;
  nh_priv.param<int>("zed_camera_number", camera_number, 0);
  cv::VideoCapture cap(0);

  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {
    ROS_ERROR("Cannot open camera");
    return 1;
  }

  

  cv::Mat frame, tmp_frame, left_frame, right_frame, gray, gray_rs;
  sensor_msgs::ImagePtr left_msg, right_msg, left_rs_msg, right_rs_msg;
  // Setup frames
  cil->header.frame_id = "/zed_left";
  cil->header.stamp = ros::Time::now();
  cir->header.frame_id = "/zed_right";
  cir->header.stamp = ros::Time::now();

  int cnt = 0;

  while (nh.ok()) {
  	cap >> frame;
  	if (!frame.empty())
  	{
  		cv::cvtColor(frame, gray, CV_BGR2GRAY);

      switch (res_)
      {
        case 0:
          cv::resize(gray, gray_rs, Size(), 0.5, 0.5, INTER_CUBIC);
          break;
        case 1:
          cv::resize(gray, gray_rs, Size(), 0.75, 0.75, INTER_CUBIC);
          break;
        case 2:
          cv::resize(gray, gray_rs, Size(), 1, 1, INTER_CUBIC);
          break;
      }

      cnt++;
      if (cnt > frameSkip)
      {
        cnt = 0;
        left_frame = gray_rs(Rect(0, 0, gray_rs.cols / 2, gray_rs.rows));
        right_frame = gray_rs(Rect(gray_rs.cols / 2, 0, gray_rs.cols / 2, gray_rs.rows));


        left_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", left_frame)
          .toImageMsg();
        right_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", right_frame)
          .toImageMsg();

        left_msg->header.frame_id = "/zed_left";
        left_msg->header.stamp = ros::Time::now();
        cil->header.frame_id = "/zed_left";
        cil->header.stamp = left_msg->header.stamp;
        right_msg->header.frame_id = "/zed_right";
        right_msg->header.stamp = left_msg->header.stamp;
        cir->header.frame_id = "/zed_right";
        cir->header.stamp = left_msg->header.stamp;

        left_pub.publish(left_msg, cil);
        right_pub.publish(right_msg, cir);
          
        ros::spinOnce();
      } 	
  	} 	
  }
}