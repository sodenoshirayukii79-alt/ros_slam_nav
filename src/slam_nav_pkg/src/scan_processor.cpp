/**
 * 激光扫描预处理节点
 * 对原始激光数据进行过滤和裁剪
 * 订阅 /scan，发布 /scan_processed
 */
#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace {

double deg2rad(double deg) { return deg * M_PI / 180.0; }

float normalizeAnglePi(float a)
{
    while (a > static_cast<float>(M_PI))
        a -= static_cast<float>(2.0 * M_PI);
    while (a < static_cast<float>(-M_PI))
        a += static_cast<float>(2.0 * M_PI);
    return a;
}

bool finiteRange(float r) { return std::isfinite(r); }

float clampRangeValue(float r, double min_d, double max_d)
{
    if (!finiteRange(r))
        return static_cast<float>(max_d);
    if (r < static_cast<float>(min_d))
        return static_cast<float>(min_d);
    if (r > static_cast<float>(max_d))
        return static_cast<float>(max_d);
    return r;
}

bool angleInFrontArc(float angle_rad, double half_arc_rad)
{
    return std::fabs(normalizeAnglePi(angle_rad)) <= half_arc_rad + 1e-6;
}

}  // 匿名命名空间

class ScanProcessorNode
{
public:
    ScanProcessorNode()
        : nh_()
        , pnh_("~")
    {
        loadParameters();

        pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, queue_size_);
        sub_ = nh_.subscribe(input_topic_, queue_size_, &ScanProcessorNode::scanCallback, this);

        ROS_INFO("scan_processor: %s -> %s (range [%.3f, %.3f] m, front_arc=%.1f deg, limit_arc=%s)",
                 input_topic_.c_str(), output_topic_.c_str(), min_range_m_, max_range_m_,
                 front_arc_deg_, limit_to_front_arc_ ? "true" : "false");
    }

private:
    void loadParameters()
    {
        pnh_.param<std::string>("input_topic", input_topic_, std::string("/scan"));
        pnh_.param<std::string>("output_topic", output_topic_, std::string("/scan_processed"));
        pnh_.param("queue_size", queue_size_, 10);

        pnh_.param("min_range", min_range_m_, 0.1);
        pnh_.param("max_range", max_range_m_, 10.0);
        if (min_range_m_ < 0.0)
        {
            ROS_WARN("~min_range < 0, clamping to 0.");
            min_range_m_ = 0.0;
        }
        if (max_range_m_ <= min_range_m_)
        {
            ROS_WARN("~max_range <= ~min_range, forcing max_range = min_range + 0.01");
            max_range_m_ = min_range_m_ + 0.01;
        }

        pnh_.param("limit_to_front_arc", limit_to_front_arc_, false);
        pnh_.param("front_arc_deg", front_arc_deg_, 180.0);
        if (front_arc_deg_ <= 0.0)
        {
            ROS_WARN("~front_arc_deg <= 0, disabling arc limit.");
            limit_to_front_arc_ = false;
        }
        if (front_arc_deg_ >= 360.0)
            limit_to_front_arc_ = false;
    }

    void filterBeam(size_t i, const sensor_msgs::LaserScan &in, sensor_msgs::LaserScan &out) const
    {
        const float angle = in.angle_min + static_cast<float>(i) * in.angle_increment;
        const float r_in = in.ranges[i];

        if (limit_to_front_arc_)
        {
            const double half_arc = 0.5 * deg2rad(front_arc_deg_);
            if (!angleInFrontArc(angle, half_arc))
            {
                out.ranges[i] = static_cast<float>(max_range_m_);
                return;
            }
        }

        out.ranges[i] = clampRangeValue(r_in, min_range_m_, max_range_m_);
    }

    sensor_msgs::LaserScan buildProcessedScan(const sensor_msgs::LaserScan &msg_in) const
    {
        sensor_msgs::LaserScan out = msg_in;
        out.range_min = static_cast<float>(min_range_m_);
        out.range_max = static_cast<float>(max_range_m_);

        const size_t n = msg_in.ranges.size();
        out.ranges.resize(n);
        if (!msg_in.intensities.empty())
        {
            out.intensities.resize(n);
            for (size_t i = 0; i < n; ++i)
                out.intensities[i] =
                    (i < msg_in.intensities.size()) ? msg_in.intensities[i] : 0.f;
        }
        else
            out.intensities.clear();

        for (size_t i = 0; i < n; ++i)
            filterBeam(i, msg_in, out);

        return out;
    }

    void scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        if (msg->ranges.empty())
        {
            ROS_WARN_THROTTLE(2.0, "scan_processor: empty LaserScan on %s", input_topic_.c_str());
            return;
        }
        if (!std::isfinite(msg->angle_increment) || std::fabs(msg->angle_increment) < 1e-12f)
        {
            ROS_WARN_THROTTLE(2.0, "scan_processor: invalid angle_increment");
            return;
        }

        const sensor_msgs::LaserScan out = buildProcessedScan(*msg);
        pub_.publish(out);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    std::string input_topic_;
    std::string output_topic_;
    int queue_size_{10};

    double min_range_m_{0.1};
    double max_range_m_{10.0};
    bool limit_to_front_arc_{false};
    double front_arc_deg_{180.0};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_processor");
    ScanProcessorNode node;
    ros::spin();
    return 0;
}
