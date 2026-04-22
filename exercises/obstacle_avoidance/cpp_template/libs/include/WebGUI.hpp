#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include "common_interfaces_cpp/hal/laser.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "Map.hpp"
#include "Lap.hpp"
#include <memory>
#include <vector>

class WebGUI : public BaseWebGUI
{
public:
    WebGUI();
    ~WebGUI() override;

    json update_gui() override;
    void process_message(const std::string& msg) override;
    std::vector<rclcpp::Node::SharedPtr> get_nodes() override;

    static void showForces(const std::vector<double>& v1, const std::vector<double>& v2, const std::vector<double>& v3);
    static void showLocalTarget(const std::vector<double>& v);
    static std::shared_ptr<Target> getNextTarget();
    static void setTargetx(double x);
    static void setTargety(double y);

private:
    void target_reached_callback(std_msgs::msg::Bool::UniquePtr msg);
    void publish_current_target();

    std::shared_ptr<OdometryNode> pose3d_node_;
    std::shared_ptr<LaserNode> laser_node_;
    std::shared_ptr<Map> map_;
    std::shared_ptr<Lap> lap_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_car_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_avg_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reached_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_current_target_;

    std::shared_ptr<Target> current_target_obj_;
    static WebGUI* instance_;
};

#endif