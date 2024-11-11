#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

class IMUReadyCheck {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher ready_pub_;
    ros::Timer timeout_timer_;

    int msg_count_;
    int required_msgs_;
    double timeout_;

public:
    IMUReadyCheck() : private_nh_("~"), msg_count_(0) {
        // 获取参数
        private_nh_.param("timeout", timeout_, 5.0);
        private_nh_.param("required_msgs", required_msgs_, 100);

        // 初始化发布者
        ready_pub_ = nh_.advertise<std_msgs::Bool>("/imu_ready", 1);

        // 设置超时定时器
        timeout_timer_ = nh_.createTimer(
            ros::Duration(timeout_),
            &IMUReadyCheck::timeoutCallback,
            this,
            true  // oneshot=true
        );

        // 订阅IMU话题
        imu_sub_ = nh_.subscribe("/imu_filtered", 10,
            &IMUReadyCheck::imuCallback, this);

        ROS_INFO("IMU ready check node initialized with:");
        ROS_INFO("Timeout: %.1f seconds", timeout_);
        ROS_INFO("Required messages: %d", required_msgs_);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        msg_count_++;

        if (msg_count_ >= required_msgs_) {
            publishReady("IMU initialization complete");
        }
    }

    void timeoutCallback(const ros::TimerEvent& event) {
        if (msg_count_ < required_msgs_) {
            ROS_WARN_STREAM("IMU initialization timeout after " << timeout_
                << " seconds. Received only " << msg_count_
                << " messages. Continuing anyway...");
            publishReady("IMU initialization timeout", false);
        }
    }

private:
    void publishReady(const std::string& message, bool success = true) {
        std_msgs::Bool ready_msg;
        ready_msg.data = true;
        ready_pub_.publish(ready_msg);

        if (success) {
            ROS_INFO_STREAM(message);
        } else {
            ROS_WARN_STREAM(message);
        }

        // 清理订阅和定时器
        imu_sub_.shutdown();
        timeout_timer_.stop();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_ready_check");
    IMUReadyCheck imu_checker;
    ros::spin();
    return 0;
}