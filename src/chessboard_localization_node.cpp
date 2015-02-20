#include "chessboard_localization/chessboard_localization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chessboard_localization");
  chessboard_localization::ChessboardLocalization cbl;
  ros::spin();
}
