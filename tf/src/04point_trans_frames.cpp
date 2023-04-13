#include "../include/04point_trans_frames.hpp"

int main(int argc, char const* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PointTransFrames>());

    rclcpp::shutdown();

    return 0;
}