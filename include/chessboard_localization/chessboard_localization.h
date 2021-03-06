#ifndef CHESSBOARD_LOCALIZATION_H
#define CHESSBOARD_LOCALIZATION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

namespace chessboard_localization
{

class ChessboardLocalization
{
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher pose_pub_;
  std::vector<cv::Point3f> board_points_;
  cv::Mat dist_coeffs_, cam_matrix_;
  tf::TransformBroadcaster tf_broadcaster_;
  int32_t c_width_, c_height_;

public:
  ChessboardLocalization();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

}  // namespace CHESSBOARD_LOCALIZATION_H


#endif  // CHESSBOARD_LOCALIZATION_H
