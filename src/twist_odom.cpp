#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <localization_msgs/srv/reset_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class OmniOdometry : public rclcpp::Node {
public:
    OmniOdometry() : Node("omni_odometry") {
        this->declare_parameter<std::string>("parant_frame", "map");
        this->declare_parameter<std::string>("frame_id", "base_footprint");
        this->declare_parameter<std::string>("twist_topic", "cmd_vel");
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("reset_service", "reset_odom");
        this->get_parameter("parant_frame", parant_frame_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("twist_topic", twist_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("reset_service", reset_service_);

        this->declare_parameter<double>("x_initial", 0.0);
        this->declare_parameter<double>("y_initial", 0.0);
        this->declare_parameter<double>("theta_initial", 0.0);

        x_ = get_parameter_or<double>("x_initial", 0.0);
        y_ = get_parameter_or<double>("y_initial", 0.0);
        theta_ = get_parameter_or<double>("theta_initial", 0.0);

        odom_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(odom_topic_, 10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_topic_, 10, std::bind(&OmniOdometry::twistCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        reset_odom_srv_ = this->create_service<localization_msgs::srv::ResetOdometry>(
            reset_service_,
            std::bind(&OmniOdometry::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
        odom_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OmniOdometry::timerCallback, this));
        last_time_ = this->get_clock()->now();
    }

private:
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    std::string parant_frame_;
    std::string frame_id_;
    std::string twist_topic_;
    std::string odom_topic_;
    std::string reset_service_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<localization_msgs::srv::ResetOdometry>::SharedPtr reset_odom_srv_;

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        odom_timer_->cancel();

        const double vx = msg->linear.x;
        const double vy = msg->linear.y;
        const double omega = msg->angular.z;

        updateOdometry(vx, vy, omega);

        if (!odom_timer_->is_ready()) {
            odom_timer_->reset();
        }
    }

    void resetOdometryCallback(const std::shared_ptr<localization_msgs::srv::ResetOdometry::Request> request,
                               std::shared_ptr<localization_msgs::srv::ResetOdometry::Response> response) {
        (void)request;
        response->x = x_;
        response->y = y_;
        response->theta = theta_;
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        updateOdometry(0.0, 0.0, 0.0);
    }

    void timerCallback() { updateOdometry(0.0, 0.0, 0.0); }
    void updateOdometry(double vx, double vy, double omega) {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        x_ += (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
        y_ += (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
        theta_ += omega * dt;

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = current_time;
        transform_stamped.header.frame_id = parant_frame_;
        transform_stamped.child_frame_id = frame_id_;

        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);

        geometry_msgs::msg::Pose odom_msg;
        odom_msg.position.x = x_;
        odom_msg.position.y = y_;
        odom_msg.orientation.x = q.x();
        odom_msg.orientation.y = q.y();
        odom_msg.orientation.z = q.z();
        odom_msg.orientation.w = q.w();
        odom_pub_->publish(odom_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
