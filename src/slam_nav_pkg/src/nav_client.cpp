/**
 * 导航客户端节点
 * 向 move_base 发送单点导航目标
 */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"nav_client");
    MoveBaseClient ac("move_base",true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("等待 move_base 动作服务器启动");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = -3.0;
    goal.target_pose.pose.position.y = 2.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("发送导航目标");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("任务完成！");
    else
        ROS_INFO("任务失败 ...");
    return 0;
}
