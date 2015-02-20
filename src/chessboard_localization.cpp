#include <sensor_msgs/image_encodings.h>
#include "chessboard_localization/chessboard_localization.h""

namespace chessboard_localization
{

ChessboardLocalization::ChessboardLocalization():it_(nh_)
{
  img_sub_ = it_.subscribe("input_image", 1, &ChessboardLocalization::imageCb, this);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose", 1);
  ptcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("chessboard_cloud", 1);

  double c_box_size;
  ros::param::param<int32_t>("chessboard_width", c_width_, 8);
  ros::param::param<int32_t>("chessboard_height", c_height_, 6);
  ros::param::param<double>("chessboard_box_size", c_box_size, 0.1);

  std::string calib_file_name;
  cv::FileStorage fs(calib_file_name, cv::FileStorage::READ);
  fs["distortion_coefficients"] >> dist_coeffs_;
  fs["camera_matrix"] >> cam_matrix_;

  int32_t k = 0;
  board_points_.resize(c_width_*c_height_);
  for (int32_t i = 0; i < c_width_; i++)
  {
    for (int32_t j = 0; j < c_height_; j++)
    {
      board_points_[k++] = cv::Point3f(i*c_box_size, j*c_box_size, 0.0);
    }
  }
}

void ChessboardLocalization::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat rvec, tvec;
  std::vector<cv::Point2f> board_corners;
  cv::Size num_corners = cv::Size(c_width_, c_height_);
  bool found = cv::findChessboardCorners(cv_ptr->image, num_corners, board_corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  cv::solvePnP(board_points_, board_corners, cam_matrix_, dist_coeffs_, rvec, tvec);

  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
}

}


