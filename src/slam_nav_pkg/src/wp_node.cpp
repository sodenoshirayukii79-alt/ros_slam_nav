#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// ========== 状态机 ==========
enum State { IDLE, NAVIGATING, RETRYING, FINISHED };

class WpNode
{
public:
    WpNode() : nh_("~")
    {
        // ===== 参数读取 =====
        nh_.param("current_wp_start", current_wp_, 1);
        nh_.param("max_waypoints",    max_wp_,     4);

        if (max_wp_ < 1)         { ROS_WARN("[WP_NODE] max_waypoints < 1, force to 1");   max_wp_ = 1; }
        if (current_wp_ < 1)     { ROS_WARN("[WP_NODE] current_wp_start < 1, force to 1"); current_wp_ = 1; }
        if (current_wp_ > max_wp_) { ROS_WARN("[WP_NODE] start > max, clamp");             current_wp_ = max_wp_; }

        // ===== 通信 =====
        nav_pub_          = nh_.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
        mission_done_pub_ = nh_.advertise<std_msgs::Bool>("/slam_nav/mission_done", 1, true);
        res_sub_          = nh_.subscribe("/waterplus/navi_result", 10,
                                          &WpNode::navResultCallback, this);

        // ===== 初始状态 =====
        current_state_ = NAVIGATING;
        retry_count_   = 0;
        retry_delay_   = 1.0;
        advance_pending_ = false;
        publishMissionDone(false);

        // ===== 等待订阅者 =====
        ros::Rate rate(10);
        while (ros::ok() && nav_pub_.getNumSubscribers() == 0)
        {
            ROS_WARN_THROTTLE(2.0, "[WP_NODE] Waiting for subscriber...");
            ros::spinOnce();
            rate.sleep();
        }

        // 订阅者连上后延迟启动（异步，不阻塞）
        init_timer_ = nh_.createTimer(
            ros::Duration(0.5),
            &WpNode::onInitTimer,
            this,
            true  // oneshot
        );
    }

    void spin() { ros::spin(); }

private:

    // ===== 工具 =====
    void publishMissionDone(bool done)
    {
        std_msgs::Bool msg;
        msg.data = done;
        mission_done_pub_.publish(msg);
    }

    void publishCurrentWaypoint()
    {
        if (nav_pub_.getNumSubscribers() == 0)
            ROS_WARN("[WP_NODE] No subscriber for waypoint topic!");

        std_msgs::String msg;
        msg.data = std::to_string(current_wp_);
        nav_pub_.publish(msg);
        ROS_INFO("[WP_NODE] Send waypoint %d (retry=%d/%d)",
                 current_wp_, retry_count_, MAX_RETRY);
    }

    // ===== 初始化定时器回调 =====
    void onInitTimer(const ros::TimerEvent&)
    {
        publishCurrentWaypoint();
    }

    // ===== 重试定时器（指数退避）=====
    void scheduleRetry()
    {
        ROS_WARN("[WP_NODE] Retry %d/%d after %.1fs",
                 retry_count_, MAX_RETRY, retry_delay_);

        retry_timer_ = nh_.createTimer(
            ros::Duration(retry_delay_),
            &WpNode::onRetryTimer,
            this,
            true  // oneshot
        );

        retry_delay_ *= 2.0;  // 1s → 2s → 4s
    }

    void onRetryTimer(const ros::TimerEvent&)
    {
        if (current_state_ != RETRYING) return;
        current_state_ = NAVIGATING;
        publishCurrentWaypoint();
    }

    // ===== 航点推进定时器（替代回调内 sleep）=====
    // 成功到达航点后，延迟 0.5s 再发下一个，
    // 用 timer 异步完成，不阻塞 spin 线程。
    void scheduleAdvance()
    {
        if (advance_pending_) return;  // 防止重复触发
        advance_pending_ = true;

        advance_timer_ = nh_.createTimer(
            ros::Duration(0.5),
            &WpNode::onAdvanceTimer,
            this,
            true  // oneshot
        );
    }

    void onAdvanceTimer(const ros::TimerEvent&)
    {
        advance_pending_ = false;
        current_state_   = NAVIGATING;
        publishCurrentWaypoint();
    }

    // ===== 导航结果回调（不做任何阻塞操作）=====
    void navResultCallback(const std_msgs::String& msg)
    {
        ROS_INFO("[WP_NODE] Received result='%s' state=%d current_wp=%d max_wp=%d",
                 msg.data.c_str(), current_state_, current_wp_, max_wp_);

        if (current_state_ == FINISHED)
        {
            ROS_INFO_THROTTLE(2.0, "[WP_NODE] Mission already finished.");
            return;
        }

        // ===== 成功 =====
        if (msg.data == "done")
        {
            ROS_INFO("[WP_NODE] Waypoint %d completed!", current_wp_);
            retry_count_ = 0;
            retry_delay_ = 1.0;

            if (current_wp_ < max_wp_)
            {
                current_wp_++;
                ROS_INFO("[WP_NODE] Advancing to waypoint %d/%d", current_wp_, max_wp_);
                scheduleAdvance();  // ✅ 异步延迟，不阻塞回调
            }
            else
            {
                ROS_INFO("[WP_NODE] === All waypoints completed! ===");
                current_state_ = FINISHED;
                publishMissionDone(true);
            }
        }
        // ===== 失败 =====
        else
        {
            ROS_ERROR("[WP_NODE] Navigation failed at WP %d, result='%s'",
                      current_wp_, msg.data.c_str());

            if (retry_count_ < MAX_RETRY)
            {
                retry_count_++;
                current_state_ = RETRYING;
                scheduleRetry();
            }
            else
            {
                ROS_ERROR("[WP_NODE] Retry limit reached at WP %d. Mission aborted.", current_wp_);
                current_state_ = FINISHED;
                publishMissionDone(false);
            }
        }
    }

    // ===== 成员变量 =====
    ros::NodeHandle nh_;
    ros::Publisher  nav_pub_;
    ros::Publisher  mission_done_pub_;
    ros::Subscriber res_sub_;
    ros::Timer      retry_timer_;
    ros::Timer      advance_timer_;
    ros::Timer      init_timer_;

    int    current_wp_;
    int    max_wp_;
    int    retry_count_;
    double retry_delay_;
    bool   advance_pending_;
    State  current_state_;

    static const int MAX_RETRY = 3;
};

// ========== main ==========
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wp_node");
    WpNode node;
    node.spin();
    return 0;
}