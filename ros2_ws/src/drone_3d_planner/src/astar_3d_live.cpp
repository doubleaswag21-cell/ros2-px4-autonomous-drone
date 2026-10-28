// Copyright (c) 2026 Ammaar Ahmed
// Project-specific ROS 2 autonomy implementation.

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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "std_msgs/msg/empty.hpp"
#include <octomap_msgs/conversions.h>
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


class AStar3DLive : public rclcpp::Node
{
public:
    AStar3DLive()
    : Node("astar_3d_live")
    {
        frame_id_ =
            declare_parameter<std::string>(
                "frame_id",
                "camera_init"
            );

        map_topic_ =
            declare_parameter<std::string>(
                "map_topic",
                "/octomap_binary"
            );

        odom_topic_ =
            declare_parameter<std::string>(
                "odom_topic",
                "/aft_mapped_to_init"
            );

        goal_topic_ =
            declare_parameter<std::string>(
                "goal_topic",
                "/selected_frontier_goal"
            );

        replan_trigger_topic_ =
            declare_parameter<std::string>(
                "replan_trigger_topic",
                "/planner_replan_now"
            );

        replan_period_sec_ =
            declare_parameter<double>(
                "replan_period_sec",
                4.0
            );

        horizontal_clearance_ =
            declare_parameter<double>(
                "horizontal_clearance",
                0.30
            );

        vertical_clearance_ =
            declare_parameter<double>(
                "vertical_clearance",
                0.20
            );

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/planned_path",
                rclcpp::QoS(1)
                    .reliable()
                    .transient_local()
            );

        auto map_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        auto input_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(5)
            )
            .best_effort();

        map_sub_ =
            create_subscription<
                octomap_msgs::msg::Octomap
            >(
                map_topic_,
                map_qos,
                std::bind(
                    &AStar3DLive::mapCallback,
                    this,
                    std::placeholders::_1
                )
            );

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry
            >(
                odom_topic_,
                input_qos,
                std::bind(
                    &AStar3DLive::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );

        goal_sub_ =
            create_subscription<
                geometry_msgs::msg::PointStamped
            >(
                goal_topic_,
                input_qos,
                std::bind(
                    &AStar3DLive::goalCallback,
                    this,
                    std::placeholders::_1
                )
            );

        replan_trigger_sub_ =
            create_subscription<std_msgs::msg::Empty>(
                replan_trigger_topic_,
                rclcpp::QoS(10).reliable(),
                std::bind(
                    &AStar3DLive::replanTriggerCallback,
                    this,
                    std::placeholders::_1
                )
            );

        const auto period =
            std::chrono::milliseconds(
                static_cast<int>(
                    std::max(
                        0.5,
                        replan_period_sec_
                    ) * 1000.0
                )
            );

        timer_ =
            create_wall_timer(
                period,
                std::bind(
                    &AStar3DLive::planTimerCallback,
                    this
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "M13D Live 3D A* Planner"
        );

        RCLCPP_INFO(
            get_logger(),
            "Map:  %s",
            map_topic_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Odom: %s",
            odom_topic_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Goal: %s",
            goal_topic_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Replan period: %.2f s",
            replan_period_sec_
        );

        RCLCPP_INFO(
            get_logger(),
            "Safety envelope: "
            "horizontal %.2f m, vertical %.2f m",
            horizontal_clearance_,
            vertical_clearance_
        );

        RCLCPP_INFO(
            get_logger(),
            "Waiting for live map, "
            "Point-LIO and frontier goal..."
        );
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

    // ==============================================================
    // M13D LIVE INPUTS
    // ==============================================================

    void mapCallback(
        const octomap_msgs::msg::Octomap::SharedPtr msg
    )
    {
        latest_map_msg_ = msg;
        map_dirty_ = true;
    }


    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg
    )
    {
        const double x =
            msg->pose.pose.position.x;

        const double y =
            msg->pose.pose.position.y;

        const double z =
            msg->pose.pose.position.z;

        if (
            !std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z)
        )
        {
            return;
        }

        latest_odom_x_ = x;
        latest_odom_y_ = y;
        latest_odom_z_ = z;

        odom_ready_ = true;
    }


    void goalCallback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg
    )
    {
        if (
            !msg->header.frame_id.empty() &&
            msg->header.frame_id != frame_id_
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring frontier goal in frame '%s'; "
                "expected '%s'",
                msg->header.frame_id.c_str(),
                frame_id_.c_str()
            );
            return;
        }

        const double x = msg->point.x;
        const double y = msg->point.y;
        const double z = msg->point.z;

        if (
            !std::isfinite(x) ||
            !std::isfinite(y) ||
            !std::isfinite(z)
        )
        {
            return;
        }

        goal_x_ = x;
        goal_y_ = y;
        goal_z_ = z;

        goal_ready_ = true;
        goal_dirty_ = true;
    }


    void replanTriggerCallback(
        const std_msgs::msg::Empty::SharedPtr /*msg*/
    )
    {
        if (
            latest_map_msg_ == nullptr ||
            !odom_ready_ ||
            !goal_ready_
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Immediate replan requested but planner inputs are not ready yet"
            );
            return;
        }

        // Force use of the latest cached map/odom/goal immediately instead of
        // waiting for the periodic fallback timer.
        map_dirty_ = true;
        goal_dirty_ = true;

        RCLCPP_INFO(
            get_logger(),
            "EVENT REPLAN requested -> planning immediately"
        );

        planTimerCallback();
    }


    void planTimerCallback()
    {
        if (
            latest_map_msg_ == nullptr ||
            !odom_ready_ ||
            !goal_ready_
        )
        {
            return;
        }

        if (
            !map_dirty_ &&
            !goal_dirty_
        )
        {
            return;
        }

        const auto msg = latest_map_msg_;

        if (!msg->binary)
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring non-binary OctoMap"
            );
            return;
        }

        if (
            !msg->header.frame_id.empty() &&
            msg->header.frame_id != frame_id_
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring OctoMap in frame '%s'; "
                "expected '%s'",
                msg->header.frame_id.c_str(),
                frame_id_.c_str()
            );
            return;
        }

        std::unique_ptr<octomap::AbstractOcTree>
            abstract_tree{
                octomap_msgs::binaryMsgToMap(*msg)
            };

        if (!abstract_tree)
        {
            RCLCPP_ERROR(
                get_logger(),
                "binaryMsgToMap() failed"
            );
            return;
        }

        auto *octree =
            dynamic_cast<octomap::OcTree *>(
                abstract_tree.get()
            );

        if (octree == nullptr)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Live map is not octomap::OcTree"
            );
            return;
        }

        tree_.reset(
            static_cast<octomap::OcTree *>(
                abstract_tree.release()
            )
        );

        resolution_ = tree_->getResolution();

        collision_cache_.clear();

        start_x_ = latest_odom_x_;
        start_y_ = latest_odom_y_;
        start_z_ = latest_odom_z_;

        map_dirty_ = false;
        goal_dirty_ = false;

        ++planning_cycle_;

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "M13D LIVE A* PLAN #%zu",
            planning_cycle_
        );

        RCLCPP_INFO(
            get_logger(),
            "Live OcTree: nodes=%zu, resolution=%.2f m",
            tree_->size(),
            resolution_
        );

        RCLCPP_INFO(
            get_logger(),
            "Point-LIO actual: [%.2f, %.2f, %.2f]",
            latest_odom_x_,
            latest_odom_y_,
            latest_odom_z_
        );

        RCLCPP_INFO(
            get_logger(),
            "Planning start:  [%.2f, %.2f, %.2f]",
            start_x_,
            start_y_,
            start_z_
        );

        RCLCPP_INFO(
            get_logger(),
            "Frontier goal:   [%.2f, %.2f, %.2f]",
            goal_x_,
            goal_y_,
            goal_z_
        );

        runAStar();

        RCLCPP_INFO(
            get_logger(),
            "M13D LIVE A* CYCLE COMPLETE"
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );
    }


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


    void runAStar()
    {
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
            // The LiDAR/vehicle can leave a small unknown near-field bubble
            // around the vehicle itself. Do NOT weaken the global collision
            // checker: instead, snap only the planning start to the nearest
            // cell that already passes the full existing clearance test.
            const double max_start_snap_m = 0.80;
            const double max_vertical_start_snap_m = 0.15;
            const int snap_voxels =
                static_cast<int>(
                    std::ceil(max_start_snap_m / resolution_));

            bool found_safe_start = false;
            octomap::OcTreeKey best_start_key = start_key;
            double best_start_distance =
                std::numeric_limits<double>::infinity();

            for (int dx = -snap_voxels; dx <= snap_voxels; ++dx)
            {
                for (int dy = -snap_voxels; dy <= snap_voxels; ++dy)
                {
                    for (int dz = -snap_voxels; dz <= snap_voxels; ++dz)
                    {
                        // Keep recovery near the vehicle's CURRENT altitude.
                        // Horizontal escape may be large, but never allow the
                        // safe-start snap to command a large dive/climb.
                        const double vertical_snap =
                            std::abs(static_cast<double>(dz) * resolution_);
                        if (vertical_snap > max_vertical_start_snap_m + 1e-6)
                            continue;

                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;

                        const double snap_distance =
                            resolution_ *
                            std::sqrt(
                                static_cast<double>(
                                    dx * dx + dy * dy + dz * dz));

                        if (snap_distance > max_start_snap_m + 1e-6)
                            continue;

                        const int nx =
                            static_cast<int>(start_key.k[0]) + dx;
                        const int ny =
                            static_cast<int>(start_key.k[1]) + dy;
                        const int nz =
                            static_cast<int>(start_key.k[2]) + dz;

                        if (
                            nx < 0 || ny < 0 || nz < 0 ||
                            nx > 65535 ||
                            ny > 65535 ||
                            nz > 65535)
                        {
                            continue;
                        }

                        octomap::OcTreeKey candidate(
                            static_cast<unsigned short>(nx),
                            static_cast<unsigned short>(ny),
                            static_cast<unsigned short>(nz));

                        if (
                            snap_distance < best_start_distance &&
                            isCollisionFree(candidate))
                        {
                            best_start_key = candidate;
                            best_start_distance = snap_distance;
                            found_safe_start = true;
                        }
                    }
                }
            }

            if (!found_safe_start)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "START has no fully-clear cell within %.2f m.",
                    max_start_snap_m);

                return;
            }

            start_key = best_start_key;

            const octomap::point3d snapped_start =
                tree_->keyToCoord(start_key);

            RCLCPP_WARN(
                get_logger(),
                "START snapped to nearest fully-clear cell: "
                "[%.2f, %.2f, %.2f], offset=%.2f m",
                snapped_start.x(),
                snapped_start.y(),
                snapped_start.z(),
                best_start_distance);
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
    // ==============================================================
    // M13D LIVE STATE
    // ==============================================================

    std::string map_topic_;
    std::string odom_topic_;
    std::string goal_topic_;
    std::string replan_trigger_topic_;

    double replan_period_sec_ = 4.0;

    bool map_dirty_ = false;
    bool goal_dirty_ = false;
    bool odom_ready_ = false;
    bool goal_ready_ = false;

    std::size_t planning_cycle_ = 0;

    double latest_odom_x_ = 0.0;
    double latest_odom_y_ = 0.0;
    double latest_odom_z_ = 0.0;

    octomap_msgs::msg::Octomap::SharedPtr
        latest_map_msg_;

    rclcpp::Subscription<
        octomap_msgs::msg::Octomap
    >::SharedPtr map_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped
    >::SharedPtr goal_sub_;

    rclcpp::Subscription<
        std_msgs::msg::Empty
    >::SharedPtr replan_trigger_sub_;


};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<AStar3DLive>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
