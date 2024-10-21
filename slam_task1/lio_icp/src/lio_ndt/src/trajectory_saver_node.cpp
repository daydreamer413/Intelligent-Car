#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <iomanip>

class TrajectorySaver
{
public:
    TrajectorySaver(ros::NodeHandle& nh, const std::string& topic_name, const std::string& file_name)
        : nh_(nh), file_name_(file_name)
    {
        odom_sub_ = nh_.subscribe(topic_name, 1000, &TrajectorySaver::odomCallback, this);
        file_.open(file_name_, std::ios::out);
        if (!file_.is_open())
        {
            ROS_ERROR("Failed to open file: %s", file_name_.c_str());
        }
        else
        {
            ROS_INFO("Saving trajectory to file: %s", file_name_.c_str());
        }
    }

    ~TrajectorySaver()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (file_.is_open())
        {
            file_ << std::fixed << std::setprecision(6)
                  << msg->header.stamp.toSec() << " "
                  << msg->pose.pose.position.x << " "
                  << msg->pose.pose.position.y << " "
                  << msg->pose.pose.position.z << " "
                  << msg->pose.pose.orientation.x << " "
                  << msg->pose.pose.orientation.y << " "
                  << msg->pose.pose.orientation.z << " "
                  << msg->pose.pose.orientation.w << std::endl;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    std::ofstream file_;
    std::string file_name_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_saver_node");
    ros::NodeHandle nh;

    std::string topic_name = "/gnss";
    std::string file_name = "/home/siai/Desktop/EIE_WORK/slam_task1/lio_icp/gnss.txt";

    TrajectorySaver trajectory_saver(nh, topic_name, file_name);

    ros::spin();

    return 0;
}