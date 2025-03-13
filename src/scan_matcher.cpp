#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "localization_msgs/msg/point_array.hpp"

using namespace std::chrono_literals;

class ScanMatcher : public rclcpp::Node {
    struct Point2D {
        double x, y;
    };

    struct Rect {
        double x_min, x_max, y_min, y_max;
    };

public:
    ScanMatcher() : Node("scan_matcher") {
        // runtime parameters
        this->declare_parameter<std::string>("input_topic", "point_array");
        this->declare_parameter<std::string>("imu_topic", "imu");
        this->declare_parameter<std::string>("target_frame", "base_footprint");
        this->declare_parameter<std::string>("output_frame", "odom");
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<double>("field_width", 8.0);
        this->declare_parameter<double>("field_height", 15.0);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("output_frame", output_frame_);
        this->get_parameter("parent_frame", parent_frame_);

        double field_width, field_height;
        this->get_parameter("field_width", field_width);
        this->get_parameter("field_height", field_height);

        const double x_min = -field_width / 2.0;
        const double x_max = field_width / 2.0;
        const double y_min = -field_height / 2.0;
        const double y_max = field_height / 2.0;
        map_ = {x_min, x_max, y_min, y_max};

        // dynamic parameters
        this->declare_parameter<double>("delta", 0.0001);
        this->declare_parameter<double>("epsilon", 1e-4);
        this->declare_parameter<double>("learning_rate", 0.1);
        this->declare_parameter<int>("max_iterations", 50);
        this->declare_parameter<double>("max_step", 0.01);
        this->declare_parameter<double>("max_step_theta", 0.01);

        input_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            input_topic_, 10, std::bind(&ScanMatcher::topicCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, std::bind(&ScanMatcher::imuCallback, this, std::placeholders::_1));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ScanMatcher::timerCallback, this));

        last_tf_.header.stamp = this->now();
        last_tf_.header.frame_id = parent_frame_;
        last_tf_.child_frame_id = output_frame_;
        last_theta_ = 0.0;
        imu_theta_ = 0.0;
        scan_theta_ = 0.0;
        imu_last_time_ = this->get_clock()->now().seconds();
    }

private:
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr input_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string input_topic_, imu_topic_, target_frame_, output_frame_, parent_frame_;
    Rect map_;
    geometry_msgs::msg::TransformStamped last_tf_;
    rclcpp::TimerBase::SharedPtr timer_;
    double last_theta_, imu_theta_, scan_theta_;
    double imu_last_time_;

    std::vector<Point2D> transformPoints(const std::vector<Point2D> &points, double dx, double dy, double dtheta) {
        std::vector<Point2D> transformed_points(points.size());
        Point2D tmp;

        if (dtheta == 0.0) {
            for (const auto &point : points) {
                tmp.x = point.x + dx;
                tmp.y = point.y + dy;
                transformed_points.push_back(tmp);
            }
            return transformed_points;
        }

        for (const auto &point : points) {
            tmp.x = std::cos(dtheta) * point.x - std::sin(dtheta) * point.y + dx;
            tmp.y = std::sin(dtheta) * point.x + std::cos(dtheta) * point.y + dy;
            transformed_points.push_back(tmp);
        }
        return transformed_points;
    }

    double calculateCost(const std::vector<Point2D> &points, const Rect &map, double dx, double dy, double dtheta) {
        double cost = 0.0;
        auto transformed = transformPoints(points, dx, dy, dtheta);
        for (const auto &point : transformed) {
            double dx = std::min(std::abs(point.x - map.x_min), std::abs(point.x - map.x_max));
            double dy = std::min(std::abs(point.y - map.y_min), std::abs(point.y - map.y_max));
            cost += std::min(dx, dy) * std::min(dx, dy);
        }
        return cost;
    }

    geometry_msgs::msg::Pose2D optimizePoint(const std::vector<Point2D> &points, const Rect &map) {
        double delta, epsilon, learning_rate, max_step, max_step_theta;
        int max_iterations;
        this->get_parameter("delta", delta);
        this->get_parameter("epsilon", epsilon);
        this->get_parameter("learning_rate", learning_rate);
        this->get_parameter("max_iterations", max_iterations);
        this->get_parameter("max_step", max_step);
        this->get_parameter("max_step_theta", max_step_theta);

        double dx = 0.0, dy = 0.0, dtheta = 0.0;

        for (int i = 0; i < max_iterations; ++i) {
            double current_cost = calculateCost(points, map, dx, dy, dtheta);
            double grad_x = (calculateCost(points, map, dx + delta, dy, dtheta) - current_cost) / delta;
            double grad_y = (calculateCost(points, map, dx, dy + delta, dtheta) - current_cost) / delta;
            double grad_theta = (calculateCost(points, map, dx, dy, dtheta + delta) - current_cost) / delta;

            dx -= std::clamp(learning_rate * grad_x, -max_step, max_step);
            dy -= std::clamp(learning_rate * grad_y, -max_step, max_step);
            dtheta -= std::clamp(learning_rate * grad_theta, -max_step_theta, max_step_theta);

            if (std::abs(grad_x) < epsilon && std::abs(grad_y) < epsilon && std::abs(grad_theta) < epsilon) {
                break;
            }

            learning_rate *= 0.9;
        }

        geometry_msgs::msg::Pose2D pose;
        pose.x = dx;
        pose.y = dy;
        pose.theta = dtheta;

        RCLCPP_INFO(this->get_logger(), "Optimized transform x: % .4f y: % .4f z: % .4f", dx, dy, dtheta);

        return pose;
    }

    void topicCallback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        std::vector<Point2D> points;
        points.reserve(msg->points.size());
        for (const auto &point : msg->points) {
            points.push_back({point.x, point.y});
        }

        rclcpp::Time start = this->get_clock()->now();
        geometry_msgs::msg::Pose2D delta = optimizePoint(points, map_);

        rclcpp::Time end = this->get_clock()->now();
        double duration = (end - start).seconds();
        RCLCPP_INFO(this->get_logger(), "Optimization duration: %.3f ms", duration * 1000.0);

        scan_theta_ += delta.theta;
        double new_theta = imu_theta_ + scan_theta_;

        Point2D new_point = {last_tf_.transform.translation.x, last_tf_.transform.translation.y};
        new_point.x += (delta.x * std::cos(delta.theta) - delta.y * std::sin(delta.theta));
        new_point.y += (delta.x * std::sin(delta.theta) + delta.y * std::cos(delta.theta));

        // publish optimized transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = parent_frame_;
        t.child_frame_id = output_frame_;
        t.transform.translation.x = new_point.x;
        t.transform.translation.y = new_point.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, new_theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
        last_tf_ = t;
        last_theta_ = new_theta;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double current_time = this->get_clock()->now().seconds();
        double dt = current_time - imu_last_time_;
        imu_last_time_ = current_time;
        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid time");
            return;
        }

        imu_theta_ += msg->angular_velocity.z * dt;
    }

    void timerCallback() {
        last_tf_.header.stamp = this->get_clock()->now();
        tf_broadcaster_->sendTransform(last_tf_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatcher>());
    rclcpp::shutdown();
    return 0;
}