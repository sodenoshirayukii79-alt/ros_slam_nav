/**
 * 激光避障行为节点
 * 基于处理后的激光数据进行简单避障
 * 订阅 /scan_processed，发布 /cmd_vel
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace {

double deg2rad(double deg) { return deg * M_PI / 180.0; }

bool indexValid(const sensor_msgs::LaserScan &scan, size_t i)
{
    if (i >= scan.ranges.size())
        return false;
    const float r = scan.ranges[i];
    if (!std::isfinite(r))
        return false;
    return r >= scan.range_min - 1e-6f && r <= scan.range_max + 1e-6f;
}

size_t angleToIndex(const sensor_msgs::LaserScan &scan, float angle_rad)
{
    const float inc = scan.angle_increment;
    if (!std::isfinite(inc) || std::fabs(inc) < 1e-12f)
        return 0;
    const float idx_f = (angle_rad - scan.angle_min) / inc;
    int idx = static_cast<int>(std::lround(idx_f));
    idx = std::max(0, std::min(idx, static_cast<int>(scan.ranges.size()) - 1));
    return static_cast<size_t>(idx);
}

}  // 匿名命名空间

enum class TurnBias
{
    PreferLeft,
    PreferRight,
    Neutral
};

class LidarBehaviorNode
{
public:
    LidarBehaviorNode()
        : nh_()
        , pnh_("~")
    {
        loadParameters();

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, queue_size_);
        scan_sub_ = nh_.subscribe(scan_topic_, queue_size_, &LidarBehaviorNode::scanCallback, this);

        ROS_INFO(
            "lidar_behavior: scan=%s cmd=%s (obs_th=%.3f m, v_f=%.3f, v_b=%.3f, w=%.3f)",
            scan_topic_.c_str(), cmd_vel_topic_.c_str(), obstacle_dist_thresh_m_, forward_speed_m_s_,
            backward_speed_m_s_, angular_speed_rad_s_);
    }

private:
    void loadParameters()
    {
        pnh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan_processed"));
        pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        pnh_.param("queue_size", queue_size_, 10);

        pnh_.param("obstacle_distance", obstacle_dist_thresh_m_, 0.45);
        pnh_.param("forward_speed", forward_speed_m_s_, 0.25);
        pnh_.param("backward_speed", backward_speed_m_s_, 0.15);
        pnh_.param("angular_speed", angular_speed_rad_s_, 0.6);

        pnh_.param("front_sector_half_deg", front_sector_half_deg_, 25.0);
        pnh_.param("side_sector_half_deg", side_sector_half_deg_, 40.0);

        if (obstacle_dist_thresh_m_ <= 0.0)
        {
            ROS_WARN("~obstacle_distance <= 0, using 0.3");
            obstacle_dist_thresh_m_ = 0.3;
        }
    }

    float sectorMinRange(const sensor_msgs::LaserScan &scan, float ang_min, float ang_max) const
    {
        if (scan.ranges.empty())
            return std::numeric_limits<float>::infinity();

        const size_t i0 = angleToIndex(scan, ang_min);
        const size_t i1 = angleToIndex(scan, ang_max);
        const size_t lo = std::min(i0, i1);
        const size_t hi = std::max(i0, i1);

        float best = std::numeric_limits<float>::infinity();
        for (size_t i = lo; i <= hi && i < scan.ranges.size(); ++i)
        {
            if (indexValid(scan, i))
                best = std::min(best, scan.ranges[i]);
        }
        return best;
    }

    float getFrontMin(const sensor_msgs::LaserScan &scan) const
    {
        const float half = static_cast<float>(deg2rad(front_sector_half_deg_));
        return sectorMinRange(scan, -half, half);
    }

    TurnBias decideTurnDirection(const sensor_msgs::LaserScan &scan) const
    {
        const float side_half = static_cast<float>(deg2rad(side_sector_half_deg_));
        const float left_min = sectorMinRange(scan, side_half * 0.25f, side_half);
        const float right_min = sectorMinRange(scan, -side_half, -side_half * 0.25f);

        if (!std::isfinite(left_min) && !std::isfinite(right_min))
            return TurnBias::Neutral;
        if (!std::isfinite(right_min))
            return TurnBias::PreferRight;
        if (!std::isfinite(left_min))
            return TurnBias::PreferLeft;

        constexpr float kEps = 0.05f;
        if (left_min > right_min + kEps)
            return TurnBias::PreferLeft;
        if (right_min > left_min + kEps)
            return TurnBias::PreferRight;
        return TurnBias::Neutral;
    }

    geometry_msgs::Twist computeCmdStraight() const
    {
        geometry_msgs::Twist t;
        t.linear.x = forward_speed_m_s_;
        t.angular.z = 0.0;
        return t;
    }

    geometry_msgs::Twist computeCmdAvoid(TurnBias bias) const
    {
        geometry_msgs::Twist t;
        t.linear.x = -std::fabs(backward_speed_m_s_);

        double w = angular_speed_rad_s_;
        switch (bias)
        {
        case TurnBias::PreferLeft:
            w = +std::fabs(angular_speed_rad_s_);
            break;
        case TurnBias::PreferRight:
            w = -std::fabs(angular_speed_rad_s_);
            break;
        case TurnBias::Neutral:
            w = +std::fabs(angular_speed_rad_s_);
            break;
        }
        t.angular.z = w;
        return t;
    }

    geometry_msgs::Twist decideTwist(const sensor_msgs::LaserScan &scan) const
    {
        const float front_min = getFrontMin(scan);
        if (!std::isfinite(front_min))
        {
            ROS_WARN_THROTTLE(2.0, "lidar_behavior: no valid front range, coast straight slowly");
            geometry_msgs::Twist t;
            t.linear.x = forward_speed_m_s_ * 0.3;
            t.angular.z = 0.0;
            return t;
        }

        if (front_min < static_cast<float>(obstacle_dist_thresh_m_))
        {
            const TurnBias bias = decideTurnDirection(scan);
            ROS_WARN_THROTTLE(0.5, "lidar_behavior: obstacle ahead (min=%.3f m < %.3f), avoid",
                             front_min, obstacle_dist_thresh_m_);
            return computeCmdAvoid(bias);
        }

        return computeCmdStraight();
    }

    void scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        const geometry_msgs::Twist cmd = decideTwist(*msg);
        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber scan_sub_;

    std::string scan_topic_;
    std::string cmd_vel_topic_;
    int queue_size_{10};

    double obstacle_dist_thresh_m_{0.45};
    double forward_speed_m_s_{0.25};
    double backward_speed_m_s_{0.15};
    double angular_speed_rad_s_{0.6};
    double front_sector_half_deg_{25.0};
    double side_sector_half_deg_{40.0};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_behavior");
    LidarBehaviorNode node;
    ros::spin();
    return 0;
}
