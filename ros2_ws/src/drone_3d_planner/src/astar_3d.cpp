// Copyright (c) 2026 Ammaar Ahmed
// Project-specific ROS 2 implementation.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <octomap/OcTree.h>


struct KeyHash
{
    std::size_t operator()(const octomap::OcTreeKey &k) const
    {
        return
            (static_cast<std::size_t>(k.k[0]) << 32) ^
            (static_cast<std::size_t>(k.k[1]) << 16) ^
            static_cast<std::size_t>(k.k[2]);
    }
};

struct KeyEqual
{
    bool operator()(
        const octomap::OcTreeKey &a,
        const octomap::OcTreeKey &b) const
    {
        return
            a.k[0] == b.k[0] &&
            a.k[1] == b.k[1] &&
            a.k[2] == b.k[2];
    }
};


struct QueueNode
{
    octomap::OcTreeKey key;
    double f;

    bool operator<(const QueueNode &other) const
    {
        return f > other.f;
    }
};


class AStar3D : public rclcpp::Node
{
public:
    AStar3D()
    : Node("astar_3d")
    {
        map_path_ = declare_parameter<std::string>(
            "map_path",
            "octomap/maps/slam_room_office_m8.bt");

        frame_id_ = declare_parameter<std::string>(
            "frame_id",
            "camera_init");

        start_x_ = declare_parameter<double>("start_x", 0.0);
        start_y_ = declare_parameter<double>("start_y", 0.0);
        start_z_ = declare_parameter<double>("start_z", 1.0);

        goal_x_ = declare_parameter<double>("goal_x", 3.0);
        goal_y_ = declare_parameter<double>("goal_y", 3.0);
        goal_z_ = declare_parameter<double>("goal_z", 1.0);

        horizontal_clearance_ =
            declare_parameter<double>(
                "horizontal_clearance", 0.30);

        vertical_clearance_ =
            declare_parameter<double>(
                "vertical_clearance", 0.20);

        tree_ = std::make_unique<octomap::OcTree>(map_path_);
        resolution_ = tree_->getResolution();

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/planned_path",
                rclcpp::QoS(1)
                    .reliable()
                    .transient_local());

        RCLCPP_INFO(
            get_logger(),
            "Loaded OctoMap: %s",
            map_path_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Resolution: %.2f m",
            resolution_);

        RCLCPP_INFO(
            get_logger(),
            "Start: [%.2f, %.2f, %.2f]",
            start_x_, start_y_, start_z_);

        RCLCPP_INFO(
            get_logger(),
            "Goal:  [%.2f, %.2f, %.2f]",
            goal_x_, goal_y_, goal_z_);
        RCLCPP_INFO(
            get_logger(),
            "Safety envelope: horizontal %.2f m, vertical %.2f m",
            horizontal_clearance_,
            vertical_clearance_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&AStar3D::runOnce, this));
    }


private:
    using KeyMapDouble =
        std::unordered_map<
            octomap::OcTreeKey,
            double,
            KeyHash,
            KeyEqual>;

    using KeyMapKey =
        std::unordered_map<
            octomap::OcTreeKey,
            octomap::OcTreeKey,
            KeyHash,
            KeyEqual>;

    bool isCollisionFree(
        const octomap::OcTreeKey &key)
    {
        // Reuse previous clearance checks.
        auto cached = collision_cache_.find(key);

        if (cached != collision_cache_.end())
            return cached->second;

        const int horizontal_voxels =
            static_cast<int>(
                std::ceil(
                    horizontal_clearance_ /
                    resolution_));
        const int vertical_voxels =
            static_cast<int>(
                std::ceil(
                    vertical_clearance_ /
                    resolution_));

        for (
            int dx = -horizontal_voxels;
            dx <= horizontal_voxels;
            ++dx)
        {
            for (
                int dy = -horizontal_voxels;
                dy <= horizontal_voxels;
                ++dy)
            {
                const double horizontal_distance =
                    resolution_ *
                    std::sqrt(
                        static_cast<double>(
                            dx * dx + dy * dy));

                if (
                    horizontal_distance >
                    horizontal_clearance_ + 1e-6)
                {
                    continue;
                }

                for (
                    int dz = -vertical_voxels;
                    dz <= vertical_voxels;
                    ++dz)
                {
                    const double vertical_distance =
                        std::abs(
                            static_cast<double>(dz) *
                            resolution_);

                    if (
                        vertical_distance >
                        vertical_clearance_ + 1e-6)
                    {
                        continue;
                    }

                    const int nx =
                        static_cast<int>(key.k[0]) + dx;

                    const int ny =
                        static_cast<int>(key.k[1]) + dy;

                    const int nz =
                        static_cast<int>(key.k[2]) + dz;

                    if (
                        nx < 0 ||
                        ny < 0 ||
                        nz < 0 ||
                        nx > 65535 ||
                        ny > 65535 ||
                        nz > 65535)
                    {
                        collision_cache_[key] = false;
                        return false;
                    }

                    octomap::OcTreeKey check_key(
                        static_cast<unsigned short>(nx),
                        static_cast<unsigned short>(ny),
                        static_cast<unsigned short>(nz));

                    octomap::OcTreeNode *node =
                        tree_->search(check_key);

                    // Unknown space remains unsafe.
                    if (node == nullptr)
                    {
                        collision_cache_[key] = false;
                        return false;
                    }

                    if (tree_->isNodeOccupied(node))
                    {
                        collision_cache_[key] = false;
                        return false;
                    }
                }
            }
        }

        collision_cache_[key] = true;
        return true;
    }

    bool isTransitionCollisionFree(
        const octomap::OcTreeKey &current,
        int dx,
        int dy,
        int dz)
    {
        /*
         * Prevent diagonal corner-cutting.
         *
         * For a diagonal move, all intermediate
         * axis/face-adjacent cells must also satisfy
         * the full drone clearance envelope.
         */

        const int xs[2] = {0, dx};
        const int ys[2] = {0, dy};
        const int zs[2] = {0, dz};

        const int nx_count =
            (dx == 0) ? 1 : 2;

        const int ny_count =
            (dy == 0) ? 1 : 2;

        const int nz_count =
            (dz == 0) ? 1 : 2;

        for (int ix = 0; ix < nx_count; ++ix)
        {
            for (int iy = 0; iy < ny_count; ++iy)
            {
                for (int iz = 0; iz < nz_count; ++iz)
                {
                    const int ox = xs[ix];
                    const int oy = ys[iy];
                    const int oz = zs[iz];
                    // Current voxel.
                    if (
                        ox == 0 &&
                        oy == 0 &&
                        oz == 0)
                    {
                        continue;
                    }

                    // Destination voxel.
                    // Checked separately by isCollisionFree().
                    if (
                        ox == dx &&
                        oy == dy &&
                        oz == dz)
                    {
                        continue;
                    }

                    const int kx =
                        static_cast<int>(
                            current.k[0]) + ox;

                    const int ky =
                        static_cast<int>(
                            current.k[1]) + oy;

                    const int kz =
                        static_cast<int>(
                            current.k[2]) + oz;

                    if (
                        kx < 0 ||
                        ky < 0 ||
                        kz < 0 ||
                        kx > 65535 ||
                        ky > 65535 ||
                        kz > 65535)
                    {
                        return false;
                    }
                    octomap::OcTreeKey intermediate(
                        static_cast<unsigned short>(kx),
                        static_cast<unsigned short>(ky),
                        static_cast<unsigned short>(kz));

                    if (!isCollisionFree(intermediate))
                        return false;
                }
            }
        }

        return true;
    }


    double heuristic(
        const octomap::OcTreeKey &a,
        const octomap::OcTreeKey &b)
    {
        const double dx =
            static_cast<double>(a.k[0]) -
            static_cast<double>(b.k[0]);

        const double dy =
            static_cast<double>(a.k[1]) -
            static_cast<double>(b.k[1]);

        const double dz =
            static_cast<double>(a.k[2]) -
            static_cast<double>(b.k[2]);

        return
            resolution_ *
            std::sqrt(
                dx * dx +
                dy * dy +
                dz * dz);
    }


    std::vector<octomap::OcTreeKey>
    reconstructPath(
        const KeyMapKey &came_from,
        octomap::OcTreeKey current,
        const octomap::OcTreeKey &start)
    {
        std::vector<octomap::OcTreeKey> path;

        path.push_back(current);

        while (!KeyEqual{}(current, start))
        {
            auto it = came_from.find(current);

            if (it == came_from.end())
                break;

            current = it->second;
            path.push_back(current);
        }

        std::reverse(
            path.begin(),
            path.end());

        return path;
    }


    bool validatePath(
        const std::vector<octomap::OcTreeKey> &keys)
    {
        if (keys.empty())
            return false;

        for (
            std::size_t i = 0;
            i < keys.size();
            ++i)
        {
            if (!isCollisionFree(keys[i]))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Validation failed: waypoint %zu lacks clearance.",
                    i);

                return false;
            }

            if (i == 0)
                continue;

            const int dx =
                static_cast<int>(keys[i].k[0]) -
                static_cast<int>(keys[i - 1].k[0]);

            const int dy =
                static_cast<int>(keys[i].k[1]) -
                static_cast<int>(keys[i - 1].k[1]);

            const int dz =
                static_cast<int>(keys[i].k[2]) -
                static_cast<int>(keys[i - 1].k[2]);

            if (
                std::abs(dx) > 1 ||
                std::abs(dy) > 1 ||
                std::abs(dz) > 1)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Validation failed: non-adjacent path nodes.");

                return false;
            }

            if (
                !isTransitionCollisionFree(
                    keys[i - 1],
                    dx,
                    dy,
                    dz))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Validation failed: unsafe transition at segment %zu.",
                    i);

                return false;
            }
        }

        return true;
    }


    void publishPath(
        const std::vector<octomap::OcTreeKey> &keys)
    {
        nav_msgs::msg::Path path;

        path.header.stamp = now();
        path.header.frame_id = frame_id_;

        double total_distance = 0.0;

        double min_z =
            std::numeric_limits<double>::infinity();

        double max_z =
            -std::numeric_limits<double>::infinity();

        octomap::point3d previous_point;

        bool have_previous = false;

        for (const auto &key : keys)
        {
            const octomap::point3d p =
                tree_->keyToCoord(key);

            min_z = std::min(
                min_z,
                static_cast<double>(p.z()));

            max_z = std::max(
                max_z,
                static_cast<double>(p.z()));

            if (have_previous)
            {
                const double dx =
                    p.x() - previous_point.x();

                const double dy =
                    p.y() - previous_point.y();

                const double dz =
                    p.z() - previous_point.z();

                total_distance +=
                    std::sqrt(
                        dx * dx +
                        dy * dy +
                        dz * dz);
            }
            previous_point = p;
            have_previous = true;

            geometry_msgs::msg::PoseStamped pose;

            pose.header = path.header;

            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();

            pose.pose.orientation.w = 1.0;

            path.poses.push_back(pose);
        }

        path_pub_->publish(path);

        RCLCPP_INFO(
            get_logger(),
            "Published path with %zu waypoints",
            path.poses.size());

        RCLCPP_INFO(
            get_logger(),
            "Path distance: %.2f m",
            total_distance);
        RCLCPP_INFO(
            get_logger(),
            "Path Z range: %.2f -> %.2f m (vertical span %.2f m)",
            min_z,
            max_z,
            max_z - min_z);
    }


    void runOnce()
    {
        timer_->cancel();

        octomap::OcTreeKey start_key;
        octomap::OcTreeKey goal_key;

        if (
            !tree_->coordToKeyChecked(
                start_x_,
                start_y_,
                start_z_,
                start_key))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Start coordinate is outside OctoMap key range.");


            return;
        }

        if (
            !tree_->coordToKeyChecked(
                goal_x_,
                goal_y_,
                goal_z_,
                goal_key))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Goal coordinate is outside OctoMap key range.");

            return;
        }

        if (!isCollisionFree(start_key))
        {
            RCLCPP_ERROR(
                get_logger(),
                "START does not have required obstacle clearance.");

            return;
        }

        if (!isCollisionFree(goal_key))
        {
            RCLCPP_ERROR(
                get_logger(),
                "GOAL does not have required obstacle clearance.");

            return;
        }
        std::priority_queue<QueueNode> open_set;

        KeyMapDouble g_score;
        KeyMapKey came_from;

        g_score[start_key] = 0.0;

        open_set.push({
            start_key,
            heuristic(
                start_key,
                goal_key)
        });

        std::size_t expanded_nodes = 0;

        const std::size_t max_expansions =
            1000000;

        while (!open_set.empty())
        {
            const octomap::OcTreeKey current =
                open_set.top().key;

            open_set.pop();

            if (KeyEqual{}(current, goal_key))
            {
                auto path =
                    reconstructPath(
                        came_from,
                        current,
                        start_key);
                RCLCPP_INFO(
                    get_logger(),
                    "A* SUCCESS");

                RCLCPP_INFO(
                    get_logger(),
                    "Expanded nodes: %zu",
                    expanded_nodes);

                RCLCPP_INFO(
                    get_logger(),
                    "Path nodes: %zu",
                    path.size());

                if (!validatePath(path))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "FINAL PATH VALIDATION: FAILED");

                    return;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "FINAL PATH VALIDATION: PASS");

                publishPath(path);

                return;
            }
            ++expanded_nodes;

            if (expanded_nodes > max_expansions)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Search aborted: exceeded maximum expansions.");

                return;
            }

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dz = -1; dz <= 1; ++dz)
                    {
                        if (
                            dx == 0 &&
                            dy == 0 &&
                            dz == 0)
                        {
                            continue;
                        }

                        const int nx =
                            static_cast<int>(
                                current.k[0]) + dx;

                        const int ny =
                            static_cast<int>(
                                current.k[1]) + dy;

                        const int nz =
                            static_cast<int>(
                                current.k[2]) + dz;
                        if (
                            nx < 0 ||
                            ny < 0 ||
                            nz < 0 ||
                            nx > 65535 ||
                            ny > 65535 ||
                            nz > 65535)
                        {
                            continue;
                        }

                        octomap::OcTreeKey neighbor(
                            static_cast<unsigned short>(nx),
                            static_cast<unsigned short>(ny),
                            static_cast<unsigned short>(nz));

                        if (!isCollisionFree(neighbor))
                            continue;

                        if (
                            !isTransitionCollisionFree(
                                current,
                                dx,
                                dy,
                                dz))
                        {
                            continue;
                        }

                        const double step_cost =
                            resolution_ *
                            std::sqrt(
                                static_cast<double>(
                                    dx * dx +
                                    dy * dy +
                                    dz * dz));

                        const double tentative_g =
                            g_score[current] +
                            step_cost;

                        auto it =
                            g_score.find(neighbor);

                        if (
                            it == g_score.end() ||
                            tentative_g < it->second)
                        {
                            came_from[neighbor] =
                                current;

                            g_score[neighbor] =
                                tentative_g;

                            const double f =
                                tentative_g +
                                heuristic(
                                    neighbor,
                                    goal_key);

                            open_set.push({
                                neighbor,
                                f
                            });
                        }
                    }
                }
            }
        }
        RCLCPP_ERROR(
            get_logger(),
            "A* FAILED: no path found.");
    }


    std::string map_path_;
    std::string frame_id_;

    double start_x_;
    double start_y_;
    double start_z_;

    double goal_x_;
    double goal_y_;
    double goal_z_;

    double horizontal_clearance_;
    double vertical_clearance_;

    double resolution_;

    std::unique_ptr<octomap::OcTree> tree_;

    std::unordered_map<
        octomap::OcTreeKey,
        bool,
        KeyHash,
        KeyEqual> collision_cache_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<AStar3D>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
