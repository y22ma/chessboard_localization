#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>
#include "chessboard_localization/chessboard_localization.h"

namespace chessboard_localization
{

ChessboardLocalization::ChessboardLocalization()
{
  // configure the message subscriber and publishers
  img_sub_ = nh_.subscribe("image_raw", 1, &ChessboardLocalization::imageCb, this);
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("pose", 1);

  // initialize parameters from the launch procedures
  double c_box_size;
  std::string calib_file_name;
  // the number of inner corner of the chessboard in width
  ros::param::param<int32_t>("~chessboard_width", c_width_, 8);
  // the number of inner corner of the chessboard in height
  ros::param::param<int32_t>("~chessboard_height", c_height_, 6);
  // the size of the box in meters
  ros::param::param<double>("~chessboard_box_size", c_box_size, 0.1);
  // path of the intrinsic calibration file
  ros::param::param<std::string>("~calib_file_name", calib_file_name, "/tmp/camera_calib.yml");

  // read in intrinsic calibration parameters for the camera
  try
  {
    cv::FileStorage fs(calib_file_name, cv::FileStorage::READ);
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs["camera_matrix"] >> cam_matrix_;
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("Failed to read calibration file, Exception msg: %s", e.what());
  }
  cam_matrix_ = cv::Mat_<double>(cam_matrix_);
  dist_coeffs_ = cv::Mat_<double>(dist_coeffs_);

  // construct a vector of 3d points for the chessboard corners in the chessboard frame
  int32_t k = 0;
  board_points_.resize(c_width_*c_height_);
  for (int32_t i = 0; i < c_width_; i++)
  {
    for (int32_t j = 0; j < c_height_; j++)
    {
      board_points_[k++] = cv::Point3f(i*c_box_size, j*c_box_size, 0.0);
    }
  }

  // open a cv window for display
  cv::namedWindow("Chessboard");
}

void ChessboardLocalization::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // convert image into opencv Mat
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
  cv::Mat frame = cv_ptr->image;

  // solve a PnP problem using extracted corners features and board points
  cv::Mat rvec(3, 3, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);
  std::vector<cv::Point2f> board_corners;
  cv::Size num_corners = cv::Size(c_width_, c_height_);
  bool found = cv::findChessboardCorners(cv_ptr->image, num_corners, board_corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  cv::drawChessboardCorners(frame, num_corners, board_corners, found);
  cv::imshow("Chessboard", frame);
  cv::waitKey(1);

  if (!found)
  {
    ROS_ERROR("Chessboard not found");
    return;
  }
  cv::solvePnP(cv::Mat(board_points_), cv::Mat(board_corners),
    cam_matrix_, dist_coeffs_, rvec, tvec, false);

  // transform cv pose format into a tf structure for visualization purposes
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);
  rot_matrix = rot_matrix.t();
  tf::Transform pose_tf;
  pose_tf.getBasis().setValue(
    rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
    rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
    rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));
  pose_tf.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
  tf_broadcaster_.sendTransform(tf::StampedTransform(pose_tf, ros::Time::now(), "chessboard", "camera"));
}

}


