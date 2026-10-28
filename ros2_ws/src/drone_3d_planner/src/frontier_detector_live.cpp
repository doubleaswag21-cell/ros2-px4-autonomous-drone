// Copyright (c) 2026 Ammaar Ahmed
// Project-specific ROS 2 autonomy implementation.

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include <octomap_msgs/conversions.h>

#include "visualization_msgs/msg/marker.hpp"

#include <octomap/OcTree.h>


struct FrontierCell
{
    int ix;
    int iy;

    double x;
    double y;
};


struct FrontierCluster
{
    std::vector<std::size_t> members;

    double centroid_x;
    double centroid_y;

    double representative_x;
    double representative_y;
};


struct GoalCandidate
{
    std::size_t cluster_index;
    std::size_t cluster_size;

    double x;
    double y;
    double z;

    double distance;
    double score;
};


class FrontierDetector : public rclcpp::Node
{
public:

    FrontierDetector()
    : Node("frontier_detector_live")
    {
        // ==========================================================
        // PARAMETERS
        // ==========================================================

        map_path_ = declare_parameter<std::string>(
            "map_path",
            "m10_large_world.bt"
        );

        map_topic_ = declare_parameter<std::string>(
            "map_topic",
            "/octomap_binary"
        );

        recompute_period_sec_ = declare_parameter<double>(
            "recompute_period_sec",
            2.0
        );

        enable_goal_selection_ = declare_parameter<bool>(
            "enable_goal_selection",
            false
        );

        frame_id_ = declare_parameter<std::string>(
            "frame_id",
            "camera_init"
        );

        slice_z_ = declare_parameter<double>(
            "slice_z",
            1.55
        );

        min_cluster_size_ = declare_parameter<int>(
            "min_cluster_size",
            10
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

        min_standoff_ =
            declare_parameter<double>(
                "min_standoff",
                0.40
            );

        max_standoff_ =
            declare_parameter<double>(
                "max_standoff",
                1.00
            );

        min_goal_distance_ =
            declare_parameter<double>(
                "min_goal_distance",
                1.00
            );

        max_goal_distance_ =
            declare_parameter<double>(
                "max_goal_distance",
                10.0
            );

        size_weight_ =
            declare_parameter<double>(
                "size_weight",
                0.60
            );

        distance_weight_ =
            declare_parameter<double>(
                "distance_weight",
                0.40
            );

        // ==========================================================
        // LIVE OCTOMAP STATE
        // ==========================================================

        // Updated from every processed /octomap_binary snapshot.
        resolution_ = 0.10;

        min_x_ = 0.0;
        min_y_ = 0.0;
        min_z_ = 0.0;

        max_x_ = 0.0;
        max_y_ = 0.0;
        max_z_ = 0.0;

        // ==========================================================
        // QoS
        // ==========================================================

        auto marker_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        auto goal_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        auto odom_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(5)
            )
            .reliable();

        auto map_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();

        // ==========================================================
        // PUBLISHERS
        // ==========================================================

        raw_frontier_pub_ =
            create_publisher<
                visualization_msgs::msg::Marker
            >(
                "/frontier_points",
                marker_qos
            );

        clustered_frontier_pub_ =
            create_publisher<
                visualization_msgs::msg::Marker
            >(
                "/frontier_cluster_points",
                marker_qos
            );

        centroid_pub_ =
            create_publisher<
                visualization_msgs::msg::Marker
            >(
                "/frontier_centroids",
                marker_qos
            );

        candidate_pub_ =
            create_publisher<
                visualization_msgs::msg::Marker
            >(
                "/frontier_goal_candidates",
                marker_qos
            );

        selected_goal_marker_pub_ =
            create_publisher<
                visualization_msgs::msg::Marker
            >(
                "/selected_frontier_goal_marker",
                marker_qos
            );

        selected_goal_pub_ =
            create_publisher<
                geometry_msgs::msg::PointStamped
            >(
                "/selected_frontier_goal",
                goal_qos
            );

        // ==========================================================
        // POINT-LIO SUBSCRIBER
        // ==========================================================

        odom_sub_ =
            create_subscription<
                nav_msgs::msg::Odometry
            >(
                "/aft_mapped_to_init",
                odom_qos,
                std::bind(
                    &FrontierDetector::odomCallback,
                    this,
                    std::placeholders::_1
                )
            );

        // ==========================================================
        // LIVE OCTOMAP SUBSCRIBER
        // ==========================================================

        map_sub_ =
            create_subscription<
                octomap_msgs::msg::Octomap
            >(
                map_topic_,
                map_qos,
                std::bind(
                    &FrontierDetector::mapCallback,
                    this,
                    std::placeholders::_1
                )
            );

        const auto recompute_period =
            std::chrono::milliseconds(
                static_cast<int>(
                    recompute_period_sec_ * 1000.0
                )
            );

        recompute_timer_ =
            create_wall_timer(
                recompute_period,
                std::bind(
                    &FrontierDetector::recomputeTimerCallback,
                    this
                )
            );

        // ==========================================================
        // INFORMATION
        // ==========================================================

        RCLCPP_INFO(
            get_logger(),
            "M13C Live Frontier Goal Selector"
        );

        RCLCPP_INFO(
            get_logger(),
            "Live map topic: %s",
            map_topic_.c_str()
        );

        RCLCPP_INFO(
            get_logger(),
            "Resolution: %.2f m",
            resolution_
        );

        RCLCPP_INFO(
            get_logger(),
            "Slice height: %.2f m",
            slice_z_
        );

        RCLCPP_INFO(
            get_logger(),
            "Clearance: horizontal %.2f m, vertical %.2f m",
            horizontal_clearance_,
            vertical_clearance_
        );

        RCLCPP_INFO(
            get_logger(),
            "Goal standoff range: %.2f -> %.2f m",
            min_standoff_,
            max_standoff_
        );

        RCLCPP_INFO(
            get_logger(),
            "Goal distance range: %.2f -> %.2f m",
            min_goal_distance_,
            max_goal_distance_
        );

        RCLCPP_INFO(
            get_logger(),
            "Waiting for live /octomap_binary and "
            "Point-LIO position..."
        );
    }


private:

    // ==============================================================
    // LIVE OCTOMAP INPUT
    // ==============================================================

    void mapCallback(
        const octomap_msgs::msg::Octomap::SharedPtr msg
    )
    {
        // Do not deserialize every ~2 Hz incoming map immediately.
        // Keep only the newest snapshot and process it on our timer.
        latest_map_msg_ = msg;
        map_dirty_ = true;
    }


    void recomputeTimerCallback()
    {
        if (
            !map_dirty_ ||
            latest_map_msg_ == nullptr
        )
        {
            return;
        }

        const auto msg =
            latest_map_msg_;

        // We are now processing the newest snapshot available.
        map_dirty_ = false;

        if (!msg->binary)
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring non-binary OctoMap message"
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
                "Ignoring OctoMap in frame '%s'; expected '%s'",
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
                "Live map is not an octomap::OcTree "
                "(message id='%s')",
                msg->id.c_str()
            );
            return;
        }

        // Ownership transfers safely from AbstractOcTree
        // to our OcTree unique_ptr.
        tree_.reset(
            static_cast<octomap::OcTree *>(
                abstract_tree.release()
            )
        );

        resolution_ =
            tree_->getResolution();

        tree_->getMetricMin(
            min_x_,
            min_y_,
            min_z_
        );
        tree_->getMetricMax(
            max_x_,
            max_y_,
            max_z_
        );

        map_ready_ = true;
        ++processed_map_count_;

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "M13C LIVE MAP UPDATE #%zu",
            processed_map_count_
        );

        RCLCPP_INFO(
            get_logger(),
            "Live OcTree: nodes=%zu, resolution=%.2f m",
            tree_->size(),
            resolution_
        );

        RCLCPP_INFO(
            get_logger(),
            "Bounds: X[%.2f, %.2f] "
            "Y[%.2f, %.2f] "
            "Z[%.2f, %.2f]",
            min_x_, max_x_,
            min_y_, max_y_,
            min_z_, max_z_
        );
        // EXACT SAME M12 frontier + clustering implementation.
        detectAndClusterFrontiers();

        // ==========================================================
        // M13C LIVE GOAL SELECTION
        // ==========================================================

        if (enable_goal_selection_)
        {
            if (!odom_ready_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Goal selection skipped: "
                    "waiting for valid Point-LIO position"
                );
            }
            else
            {
                // M12C used goal_selected_ as a one-shot latch.
                // For M13C, every newly processed live map gets a
                // fresh opportunity to choose the best current goal.
                goal_selected_ = false;

                ++goal_selection_cycle_;

                RCLCPP_INFO(
                    get_logger(),
                    "M13C GOAL SELECTION CYCLE #%zu",
                    goal_selection_cycle_
                );

                RCLCPP_INFO(
                    get_logger(),
                    "Current drone position: "
                    "[%.2f, %.2f, %.2f]",
                    latest_odom_x_,
                    latest_odom_y_,
                    latest_odom_z_
                );

                selectGoal(
                    latest_odom_x_,
                    latest_odom_y_,
                    latest_odom_z_
                );

                if (goal_selected_)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "M13C goal selection: SUCCESS"
                    );
                }
                else
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "M13C goal selection: "
                        "no valid candidate this cycle"
                    );
                }
            }
        }
        RCLCPP_INFO(
            get_logger(),
            "M13C LIVE FRONTIER/GOAL UPDATE COMPLETE"
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );
    }


    // ==============================================================
    // MAP CLASSIFICATION
    // ==============================================================

    bool isFree(
        double x,
        double y,
        double z
    ) const
    {
        octomap::OcTreeNode *node =
            tree_->search(
                x,
                y,
                z
            );

        if (node == nullptr)
        {
            return false;
        }

        return !tree_->isNodeOccupied(node);
    }


    bool isUnknown(
        double x,
        double y,
        double z
    ) const
    {
        return (
            tree_->search(
                x,
                y,
                z
            ) == nullptr
        );
    }


    // ==============================================================
    // FRONTIER TEST
    // ==============================================================

    bool isFrontier(
        double x,
        double y,
        double z
    ) const
    {
        if (!isFree(x, y, z))
        {
            return false;
        }

        const double r =
            resolution_;

        const double neighbors[4][2] =
        {
            { r,  0.0 },
            {-r,  0.0 },
            {0.0,  r  },
            {0.0, -r  }
        };

        for (const auto &offset : neighbors)
        {
            if (
                isUnknown(
                    x + offset[0],
                    y + offset[1],
                    z
                )
            )
            {
                return true;
            }
        }

        return false;
    }


    // ==============================================================
    // CLEARANCE CHECK
    // ==============================================================

    bool hasRequiredClearance(
        double x,
        double y,
        double z
    ) const
    {
        // Candidate itself must be known free.

        if (!isFree(x, y, z))
        {
            return false;
        }

        const double epsilon =
            resolution_ * 0.25;

        // Check cylindrical horizontal footprint
        // through the required vertical envelope.

        for (
            double dz = -vertical_clearance_;
            dz <= vertical_clearance_ + epsilon;
            dz += resolution_
        )
        {
            for (
                double dx = -horizontal_clearance_;
                dx <= horizontal_clearance_ + epsilon;
                dx += resolution_
            )
            {
                for (
                    double dy = -horizontal_clearance_;
                    dy <= horizontal_clearance_ + epsilon;
                    dy += resolution_
                )
                {
                    const double horizontal_distance =
                        std::sqrt(
                            dx * dx +
                            dy * dy
                        );

                    if (
                        horizontal_distance >
                        horizontal_clearance_ + epsilon
                    )
                    {
                        continue;
                    }

                    const double qx =
                        x + dx;

                    const double qy =
                        y + dy;

                    const double qz =
                        z + dz;

                    octomap::OcTreeNode *node =
                        tree_->search(
                            qx,
                            qy,
                            qz
                        );

                    // Unknown is treated as unsafe,
                    // consistent with our A* philosophy.

                    if (node == nullptr)
                    {
                        return false;
                    }

                    if (
                        tree_->isNodeOccupied(node)
                    )
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }


    // ==============================================================
    // GRID KEY
    // ==============================================================

    std::int64_t gridKey(
        int ix,
        int iy
    ) const
    {
        return (
            (
                static_cast<std::int64_t>(ix)
                << 32
            )
            ^
            static_cast<std::uint32_t>(iy)
        );
    }


    // ==============================================================
    // CLUSTERING
    // ==============================================================

    std::vector<FrontierCluster>
    clusterFrontiers(
        const std::vector<FrontierCell> &cells,
        const std::unordered_map<
            std::int64_t,
            std::size_t
        > &cell_lookup
    )
    {
        std::vector<FrontierCluster>
            clusters;

        std::unordered_set<std::int64_t>
            visited;

        for (
            std::size_t start_index = 0;
            start_index < cells.size();
            ++start_index
        )
        {
            const FrontierCell &start =
                cells[start_index];

            const std::int64_t start_key =
                gridKey(
                    start.ix,
                    start.iy
                );

            if (
                visited.find(start_key)
                != visited.end()
            )
            {
                continue;
            }

            std::queue<std::size_t>
                queue;

            std::vector<std::size_t>
                members;

            queue.push(
                start_index
            );

            visited.insert(
                start_key
            );

            while (!queue.empty())
            {
                const std::size_t current_index =
                    queue.front();

                queue.pop();

                members.push_back(
                    current_index
                );

                const FrontierCell &current =
                    cells[current_index];

                for (
                    int dx = -1;
                    dx <= 1;
                    ++dx
                )
                {
                    for (
                        int dy = -1;
                        dy <= 1;
                        ++dy
                    )
                    {
                        if (
                            dx == 0 &&
                            dy == 0
                        )
                        {
                            continue;
                        }

                        const int nx =
                            current.ix + dx;

                        const int ny =
                            current.iy + dy;

                        const std::int64_t neighbor_key =
                            gridKey(
                                nx,
                                ny
                            );

                        auto found =
                            cell_lookup.find(
                                neighbor_key
                            );

                        if (
                            found ==
                            cell_lookup.end()
                        )
                        {
                            continue;
                        }

                        if (
                            visited.find(
                                neighbor_key
                            )
                            != visited.end()
                        )
                        {
                            continue;
                        }

                        visited.insert(
                            neighbor_key
                        );

                        queue.push(
                            found->second
                        );
                    }
                }
            }

            if (
                static_cast<int>(
                    members.size()
                )
                <
                min_cluster_size_
            )
            {
                continue;
            }

            double sum_x = 0.0;
            double sum_y = 0.0;

            for (
                const std::size_t index :
                members
            )
            {
                sum_x +=
                    cells[index].x;

                sum_y +=
                    cells[index].y;
            }

            const double centroid_x =
                sum_x /
                static_cast<double>(
                    members.size()
                );

            const double centroid_y =
                sum_y /
                static_cast<double>(
                    members.size()
                );

            double best_distance =
                std::numeric_limits<double>::max();

            double representative_x =
                cells[
                    members.front()
                ].x;

            double representative_y =
                cells[
                    members.front()
                ].y;

            for (
                const std::size_t index :
                members
            )
            {
                const double dx =
                    cells[index].x
                    - centroid_x;

                const double dy =
                    cells[index].y
                    - centroid_y;

                const double distance =
                    dx * dx +
                    dy * dy;

                if (
                    distance <
                    best_distance
                )
                {
                    best_distance =
                        distance;

                    representative_x =
                        cells[index].x;

                    representative_y =
                        cells[index].y;
                }
            }

            FrontierCluster cluster;

            cluster.members =
                std::move(members);

            cluster.centroid_x =
                centroid_x;

            cluster.centroid_y =
                centroid_y;

            cluster.representative_x =
                representative_x;

            cluster.representative_y =
                representative_y;

            clusters.push_back(
                std::move(cluster)
            );
        }

        return clusters;
    }


    // ==============================================================
    // FIND SAFE STANDOFF POINT
    // ==============================================================

    bool findSafeStandoff(
        double frontier_x,
        double frontier_y,
        double &goal_x,
        double &goal_y
    ) const
    {
        double best_distance =
            std::numeric_limits<double>::max();

        bool found = false;

        for (
            double dx = -max_standoff_;
            dx <= max_standoff_ + resolution_ * 0.25;
            dx += resolution_
        )
        {
            for (
                double dy = -max_standoff_;
                dy <= max_standoff_ + resolution_ * 0.25;
                dy += resolution_
            )
            {
                const double distance =
                    std::sqrt(
                        dx * dx +
                        dy * dy
                    );

                if (
                    distance <
                    min_standoff_
                )
                {
                    continue;
                }

                if (
                    distance >
                    max_standoff_
                )
                {
                    continue;
                }

                const double candidate_x =
                    frontier_x + dx;

                const double candidate_y =
                    frontier_y + dy;

                if (
                    !hasRequiredClearance(
                        candidate_x,
                        candidate_y,
                        slice_z_
                    )
                )
                {
                    continue;
                }

                // Choose the nearest safe point that
                // achieves the requested standoff.

                if (
                    distance <
                    best_distance
                )
                {
                    best_distance =
                        distance;

                    goal_x =
                        candidate_x;

                    goal_y =
                        candidate_y;

                    found = true;
                }
            }
        }

        return found;
    }


    // ==============================================================
    // DETECT + CLUSTER
    // ==============================================================

    void detectAndClusterFrontiers()
    {
        frontier_cells_.clear();
        clusters_.clear();

        std::unordered_map<
            std::int64_t,
            std::size_t
        > cell_lookup;

        const int nx =
            static_cast<int>(
                std::floor(
                    (max_x_ - min_x_)
                    / resolution_
                )
            ) + 1;

        const int ny =
            static_cast<int>(
                std::floor(
                    (max_y_ - min_y_)
                    / resolution_
                )
            ) + 1;

        std::size_t scanned_cells = 0;
        std::size_t known_free_cells = 0;

        for (
            int ix = 0;
            ix < nx;
            ++ix
        )
        {
            const double x =
                min_x_
                +
                static_cast<double>(ix)
                * resolution_;

            for (
                int iy = 0;
                iy < ny;
                ++iy
            )
            {
                const double y =
                    min_y_
                    +
                    static_cast<double>(iy)
                    * resolution_;

                ++scanned_cells;

                if (
                    !isFree(
                        x,
                        y,
                        slice_z_
                    )
                )
                {
                    continue;
                }

                ++known_free_cells;

                if (
                    !isFrontier(
                        x,
                        y,
                        slice_z_
                    )
                )
                {
                    continue;
                }

                FrontierCell cell;

                cell.ix = ix;
                cell.iy = iy;

                cell.x = x;
                cell.y = y;

                const std::size_t index =
                    frontier_cells_.size();

                frontier_cells_.push_back(
                    cell
                );

                cell_lookup[
                    gridKey(ix, iy)
                ] = index;
            }
        }

        clusters_ =
            clusterFrontiers(
                frontier_cells_,
                cell_lookup
            );

        publishFrontierMarkers();

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "M12B DATA READY FOR M12C"
        );

        RCLCPP_INFO(
            get_logger(),
            "Scanned cells: %zu",
            scanned_cells
        );

        RCLCPP_INFO(
            get_logger(),
            "Known free cells: %zu",
            known_free_cells
        );

        RCLCPP_INFO(
            get_logger(),
            "Raw frontier cells: %zu",
            frontier_cells_.size()
        );

        RCLCPP_INFO(
            get_logger(),
            "Retained clusters: %zu",
            clusters_.size()
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );
    }


    // ==============================================================
    // FRONTIER MARKERS
    // ==============================================================

    void publishFrontierMarkers()
    {
        // ----------------------------------------------------------
        // RAW FRONTIERS — ORANGE
        // ----------------------------------------------------------

        visualization_msgs::msg::Marker raw;

        raw.header.frame_id =
            frame_id_;

        raw.header.stamp =
            now();

        raw.ns =
            "raw_frontiers";

        raw.id = 0;

        raw.type =
            visualization_msgs::msg::Marker::POINTS;

        raw.action =
            visualization_msgs::msg::Marker::ADD;

        raw.pose.orientation.w = 1.0;

        raw.scale.x =
            resolution_ * 1.5;

        raw.scale.y =
            resolution_ * 1.5;

        raw.color.r = 1.0;
        raw.color.g = 0.2;
        raw.color.b = 0.0;
        raw.color.a = 1.0;

        for (
            const FrontierCell &cell :
            frontier_cells_
        )
        {
            geometry_msgs::msg::Point p;

            p.x = cell.x;
            p.y = cell.y;
            p.z = slice_z_;

            raw.points.push_back(p);
        }

        raw_frontier_pub_->publish(raw);

        // ----------------------------------------------------------
        // RETAINED CLUSTERS — GREEN
        // ----------------------------------------------------------

        visualization_msgs::msg::Marker clustered;

        clustered.header.frame_id =
            frame_id_;

        clustered.header.stamp =
            now();

        clustered.ns =
            "clustered_frontiers";

        clustered.id = 0;

        clustered.type =
            visualization_msgs::msg::Marker::POINTS;

        clustered.action =
            visualization_msgs::msg::Marker::ADD;

        clustered.pose.orientation.w = 1.0;

        clustered.scale.x =
            resolution_ * 1.7;

        clustered.scale.y =
            resolution_ * 1.7;

        clustered.color.r = 0.0;
        clustered.color.g = 1.0;
        clustered.color.b = 0.1;
        clustered.color.a = 1.0;

        // ----------------------------------------------------------
        // REPRESENTATIVES — YELLOW
        // ----------------------------------------------------------

        visualization_msgs::msg::Marker centroids;

        centroids.header.frame_id =
            frame_id_;

        centroids.header.stamp =
            now();

        centroids.ns =
            "frontier_representatives";

        centroids.id = 0;

        centroids.type =
            visualization_msgs::msg::Marker::SPHERE_LIST;

        centroids.action =
            visualization_msgs::msg::Marker::ADD;

        centroids.pose.orientation.w = 1.0;

        centroids.scale.x = 0.25;
        centroids.scale.y = 0.25;
        centroids.scale.z = 0.25;

        centroids.color.r = 1.0;
        centroids.color.g = 1.0;
        centroids.color.b = 0.0;
        centroids.color.a = 1.0;

        for (
            const FrontierCluster &cluster :
            clusters_
        )
        {
            for (
                const std::size_t member :
                cluster.members
            )
            {
                geometry_msgs::msg::Point p;

                p.x =
                    frontier_cells_[member].x;

                p.y =
                    frontier_cells_[member].y;

                p.z =
                    slice_z_ + 0.02;

                clustered.points.push_back(p);
            }

            geometry_msgs::msg::Point p;

            p.x =
                cluster.representative_x;

            p.y =
                cluster.representative_y;

            p.z =
                slice_z_ + 0.15;

            centroids.points.push_back(p);
        }

        clustered_frontier_pub_->publish(
            clustered
        );

        centroid_pub_->publish(
            centroids
        );
    }


    // ==============================================================
    // POINT-LIO CALLBACK
    // ==============================================================

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

        // M13C:
        // Keep the newest valid Point-LIO position.
        // Goal selection is synchronized to live-map updates,
        // not to the much faster odometry stream.
        latest_odom_x_ = x;
        latest_odom_y_ = y;
        latest_odom_z_ = z;

        odom_ready_ = true;
    }


    // ==============================================================
    // GOAL SELECTION
    // ==============================================================

    void selectGoal(
        double robot_x,
        double robot_y,
        double robot_z
    )
    {
        std::vector<GoalCandidate>
            candidates;

        std::size_t rejected_no_standoff = 0;
        std::size_t rejected_distance = 0;

        for (
            std::size_t i = 0;
            i < clusters_.size();
            ++i
        )
        {
            const FrontierCluster &cluster =
                clusters_[i];

            double goal_x = 0.0;
            double goal_y = 0.0;

            if (
                !findSafeStandoff(
                    cluster.representative_x,
                    cluster.representative_y,
                    goal_x,
                    goal_y
                )
            )
            {
                ++rejected_no_standoff;
                continue;
            }

            const double dx =
                goal_x - robot_x;

            const double dy =
                goal_y - robot_y;

            const double distance =
                std::sqrt(
                    dx * dx +
                    dy * dy
                );

            if (
                distance <
                min_goal_distance_
                ||
                distance >
                max_goal_distance_
            )
            {
                ++rejected_distance;
                continue;
            }

            GoalCandidate candidate;

            candidate.cluster_index = i;

            candidate.cluster_size =
                cluster.members.size();

            candidate.x = goal_x;
            candidate.y = goal_y;
            candidate.z = slice_z_;

            candidate.distance =
                distance;

            candidate.score = 0.0;

            candidates.push_back(
                candidate
            );
        }

        // ----------------------------------------------------------
        // No valid candidate
        // ----------------------------------------------------------

        if (candidates.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "M12C: no safe frontier goals found"
            );

            RCLCPP_ERROR(
                get_logger(),
                "Rejected: no safe standoff=%zu, "
                "distance=%zu",
                rejected_no_standoff,
                rejected_distance
            );

            return;
        }

        // ----------------------------------------------------------
        // Normalize cluster-size score
        // ----------------------------------------------------------

        std::size_t max_cluster_size = 1;

        for (
            const GoalCandidate &candidate :
            candidates
        )
        {
            max_cluster_size =
                std::max(
                    max_cluster_size,
                    candidate.cluster_size
                );
        }

        for (
            GoalCandidate &candidate :
            candidates
        )
        {
            const double size_ratio =
                static_cast<double>(
                    candidate.cluster_size
                )
                /
                static_cast<double>(
                    max_cluster_size
                );

            // sqrt prevents one huge frontier from
            // dominating the entire score.

            const double size_score =
                std::sqrt(
                    size_ratio
                );

            double distance_score =
                1.0
                -
                (
                    candidate.distance
                    /
                    max_goal_distance_
                );

            if (distance_score < 0.0)
            {
                distance_score = 0.0;
            }

            if (distance_score > 1.0)
            {
                distance_score = 1.0;
            }

            candidate.score =
                size_weight_
                * size_score
                +
                distance_weight_
                * distance_score;
        }

        // ----------------------------------------------------------
        // Highest score first
        // ----------------------------------------------------------

        std::sort(
            candidates.begin(),
            candidates.end(),
            [](
                const GoalCandidate &a,
                const GoalCandidate &b
            )
            {
                return (
                    a.score >
                    b.score
                );
            }
        );

        publishCandidateMarkers(
            candidates
        );

        const GoalCandidate &selected =
            candidates.front();

        // ----------------------------------------------------------
        // Publish selected goal
        // ----------------------------------------------------------

        geometry_msgs::msg::PointStamped goal;

        goal.header.frame_id =
            frame_id_;

        goal.header.stamp =
            now();

        goal.point.x =
            selected.x;

        goal.point.y =
            selected.y;

        goal.point.z =
            selected.z;

        selected_goal_pub_->publish(
            goal
        );

        publishSelectedGoalMarker(
            selected
        );

        goal_selected_ = true;

        // ----------------------------------------------------------
        // Diagnostics
        // ----------------------------------------------------------

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            get_logger(),
            "M12C SAFE FRONTIER GOAL SELECTED"
        );

        RCLCPP_INFO(
            get_logger(),
            "Current Point-LIO: "
            "[%.2f, %.2f, %.2f]",
            robot_x,
            robot_y,
            robot_z
        );

        RCLCPP_INFO(
            get_logger(),
            "Clusters available: %zu",
            clusters_.size()
        );

        RCLCPP_INFO(
            get_logger(),
            "Safe candidates: %zu",
            candidates.size()
        );

        RCLCPP_INFO(
            get_logger(),
            "Rejected - no safe standoff: %zu",
            rejected_no_standoff
        );

        RCLCPP_INFO(
            get_logger(),
            "Rejected - distance limits: %zu",
            rejected_distance
        );

        const std::size_t show_count =
            std::min<std::size_t>(
                5,
                candidates.size()
            );

        for (
            std::size_t i = 0;
            i < show_count;
            ++i
        )
        {
            const auto &candidate =
                candidates[i];

            RCLCPP_INFO(
                get_logger(),
                "Rank %zu: cluster=%zu "
                "cells=%zu "
                "goal=[%.2f, %.2f, %.2f] "
                "distance=%.2f m "
                "score=%.3f",
                i + 1,
                candidate.cluster_index + 1,
                candidate.cluster_size,
                candidate.x,
                candidate.y,
                candidate.z,
                candidate.distance,
                candidate.score
            );
        }

        RCLCPP_INFO(
            get_logger(),
            "SELECTED: cluster=%zu, "
            "goal=[%.2f, %.2f, %.2f], "
            "distance=%.2f m, "
            "score=%.3f",
            selected.cluster_index + 1,
            selected.x,
            selected.y,
            selected.z,
            selected.distance,
            selected.score
        );

        RCLCPP_INFO(
            get_logger(),
            "Published: /selected_frontier_goal"
        );

        RCLCPP_INFO(
            get_logger(),
            "========================================"
        );
    }


    // ==============================================================
    // CANDIDATE MARKERS
    // ==============================================================

    void publishCandidateMarkers(
        const std::vector<GoalCandidate> &candidates
    )
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id =
            frame_id_;

        marker.header.stamp =
            now();

        marker.ns =
            "safe_frontier_goal_candidates";

        marker.id = 0;

        marker.type =
            visualization_msgs::msg::Marker::SPHERE_LIST;

        marker.action =
            visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;

        // Cyan
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (
            const GoalCandidate &candidate :
            candidates
        )
        {
            geometry_msgs::msg::Point p;

            p.x =
                candidate.x;

            p.y =
                candidate.y;

            p.z =
                candidate.z + 0.10;

            marker.points.push_back(p);
        }

        candidate_pub_->publish(
            marker
        );
    }


    // ==============================================================
    // SELECTED GOAL MARKER
    // ==============================================================

    void publishSelectedGoalMarker(
        const GoalCandidate &selected
    )
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id =
            frame_id_;

        marker.header.stamp =
            now();

        marker.ns =
            "selected_frontier_goal";

        marker.id = 0;

        marker.type =
            visualization_msgs::msg::Marker::SPHERE;

        marker.action =
            visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x =
            selected.x;

        marker.pose.position.y =
            selected.y;

        marker.pose.position.z =
            selected.z + 0.20;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.40;
        marker.scale.y = 0.40;
        marker.scale.z = 0.40;

        // Magenta
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        selected_goal_marker_pub_->publish(
            marker
        );
    }


    // ==============================================================
    // MEMBERS
    // ==============================================================

    std::string map_path_;
    std::string map_topic_;
    std::string frame_id_;

    double recompute_period_sec_;
    bool enable_goal_selection_;

    double slice_z_;
    double resolution_;

    int min_cluster_size_;

    double horizontal_clearance_;
    double vertical_clearance_;

    double min_standoff_;
    double max_standoff_;

    double min_goal_distance_;
    double max_goal_distance_;

    double size_weight_;
    double distance_weight_;

    double min_x_;
    double min_y_;
    double min_z_;

    double max_x_;
    double max_y_;
    double max_z_;

    bool goal_selected_ = false;
    bool map_ready_ = false;
    bool map_dirty_ = false;
    std::size_t processed_map_count_ = 0;
    std::size_t goal_selection_cycle_ = 0;

    bool odom_ready_ = false;

    double latest_odom_x_ = 0.0;
    double latest_odom_y_ = 0.0;
    double latest_odom_z_ = 0.0;

    octomap_msgs::msg::Octomap::SharedPtr
        latest_map_msg_;

    std::unique_ptr<octomap::OcTree>
        tree_;

    std::vector<FrontierCell>
        frontier_cells_;

    std::vector<FrontierCluster>
        clusters_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr raw_frontier_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr clustered_frontier_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr centroid_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr candidate_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker
    >::SharedPtr selected_goal_marker_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped
    >::SharedPtr selected_goal_pub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;

    rclcpp::Subscription<
        octomap_msgs::msg::Octomap
    >::SharedPtr map_sub_;

    rclcpp::TimerBase::SharedPtr
        recompute_timer_;
};


int main(
    int argc,
    char **argv
)
{
    rclcpp::init(
        argc,
        argv
    );

    rclcpp::spin(
        std::make_shared<
            FrontierDetector
        >()
    );

    rclcpp::shutdown();

    return 0;
}
