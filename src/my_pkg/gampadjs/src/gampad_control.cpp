#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class GampadControl
{
public:
    GampadControl(): nh_("~"){

        nh_.param("axis_angular", axis_angular_, 0);      // 转向轴索引
        nh_.param("axis_linear_forward", axis_linear_forward_, 4);  // 前进轴
        nh_.param("axis_linear_backward", axis_linear_backward_, 5); // 后退轴
        nh_.param("max_linear_speed", max_linear_speed_, 0.15);  // 最大线速度(m/s)
        nh_.param("max_angular_speed", max_angular_speed_, 1.57); // 最大角速度(rad/s)
        nh_.param("deadzone", deadzone_, 0.1);            // 摇杆死区


        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &GampadControl::joyCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    };

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        geometry_msgs::Twist twist;

        // 处理线速度（前后扳机控制）
        float forward = convertTrigger(msg->axes[axis_linear_forward_]);  // 前扳机
        float backward = convertTrigger(msg->axes[axis_linear_backward_]); // 后扳机
        twist.linear.x = (forward - backward) * max_linear_speed_;

        // 处理角速度（转向轴）
        float angular_raw = msg->axes[axis_angular_];
        if(fabs(angular_raw) < deadzone_) angular_raw = 0.0; // 死区处理
        twist.angular.z = angular_raw * max_angular_speed_; 

        // 发布控制指令
        cmd_vel_pub_.publish(twist);

    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher cmd_vel_pub_;

    // 参数变量
    int axis_angular_;
    int axis_linear_forward_;
    int axis_linear_backward_;
    double max_linear_speed_;
    double max_angular_speed_;
    double deadzone_;

    float convertTrigger(float value) {
        return (1.0 - std::max(value, -1.0f)) / 2.0;  // 映射到0~1范围
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gampad_control");
    GampadControl gampad_control;

    ros::spin();

    return 0;
}