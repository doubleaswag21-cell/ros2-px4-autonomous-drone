// Copyright (c) 2026 Ammaar Ahmed
// Project-specific ROS 2 implementation.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <octomap/OcTree.h>

class OctomapQuery : public rclcpp::Node
{
public:
    OctomapQuery() : Node("octomap_query")
    {
        map_path_ = this->declare_parameter<std::string>(
            "map_path",
            "octomap/maps/slam_room_office_m8.bt"
        );

        map_frame_ = this->declare_parameter<std::string>(
            "map_frame",
            "camera_init"
        );
        try
        {
            tree_ = std::make_unique<octomap::OcTree>(map_path_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load OctoMap: %s",
                e.what()
            );
            throw;
        }

        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        tree_->getMetricMin(min_x, min_y, min_z);
        tree_->getMetricMax(max_x, max_y, max_z);

        RCLCPP_INFO(this->get_logger(),
                    "M9B OctoMap loaded successfully");

        RCLCPP_INFO(this->get_logger(),
                    "Map: %s",
                    map_path_.c_str());

        RCLCPP_INFO(this->get_logger(),
                    "Resolution: %.3f m",
                    tree_->getResolution());
        RCLCPP_INFO(
            this->get_logger(),
            "Bounds: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
            min_x, max_x,
            min_y, max_y,
            min_z, max_z
        );

        clicked_point_sub_ =
            this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point",
                10,
                std::bind(
                    &OctomapQuery::clickedPointCallback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for points on /clicked_point..."
        );
    }
private:
    void clickedPointCallback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        const double x = msg->point.x;
        const double y = msg->point.y;
        const double z = msg->point.z;

        octomap::OcTreeNode *node = tree_->search(x, y, z);

        if (node == nullptr)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[%.2f, %.2f, %.2f] -> UNKNOWN",
                x, y, z
            );
            return;
        }

        if (tree_->isNodeOccupied(node))
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[%.2f, %.2f, %.2f] -> OCCUPIED",
                x, y, z
            );
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[%.2f, %.2f, %.2f] -> FREE",
                x, y, z
            );
        }
    }

    std::string map_path_;
    std::string map_frame_;

    std::unique_ptr<octomap::OcTree> tree_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped
    >::SharedPtr clicked_point_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapQuery>());
    rclcpp::shutdown();
    return 0;
}
