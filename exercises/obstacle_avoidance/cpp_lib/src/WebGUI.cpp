#include "WebGUI.hpp"
#include <chrono>
#include <thread>
#include <iostream>

using std::placeholders::_1;

WebGUI* WebGUI::instance_ = nullptr;

WebGUI::WebGUI() : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 30.0, "/stats") {
    instance_ = this;
    pose3d_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom_node");
    laser_node_ = std::make_shared<LaserNode>("/f1/laser/scan");
    map_ = std::make_shared<Map>([this]() { return laser_node_->getLaserData(); }, [this]() { return pose3d_node_->getPose3d(); });
    lap_ = std::make_shared<Lap>(pose3d_node_);

    auto cb_car = [this](geometry_msgs::msg::Point::UniquePtr m) { map_->setCar(m->x, m->y); };
    auto cb_obs = [this](geometry_msgs::msg::Point::UniquePtr m) { map_->setObs(m->x, m->y); };
    auto cb_avg = [this](geometry_msgs::msg::Point::UniquePtr m) { map_->setAvg(m->x, m->y); };
    auto cb_target = [this](geometry_msgs::msg::Point::UniquePtr m) { map_->setTargetPos(m->x, m->y); };

    sub_car_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/car", 10, cb_car);
    sub_obs_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/obs", 10, cb_obs);
    sub_avg_ = create_subscription<geometry_msgs::msg::Point>("/webgui/force/avg", 10, cb_avg);
    sub_target_ = create_subscription<geometry_msgs::msg::Point>("/webgui/local_target", 10, cb_target);
    sub_reached_ = create_subscription<std_msgs::msg::Bool>("/webgui/target_reached", 10, std::bind(&WebGUI::target_reached_callback, this, _1));

    rclcpp::QoS qos(1); qos.transient_local();
    pub_current_target_ = create_publisher<geometry_msgs::msg::Point>("/webgui/current_target", qos);

    current_target_obj_ = map_->getNextTarget();
    publish_current_target();
}

WebGUI::~WebGUI() { if (instance_ == this) instance_ = nullptr; }

std::vector<rclcpp::Node::SharedPtr> WebGUI::get_nodes() {
    auto nodes = BaseWebGUI::get_nodes();
    nodes.push_back(pose3d_node_);
    nodes.push_back(laser_node_);
    return nodes;
}

void WebGUI::process_message(const std::string& msg) {
    if (msg.find("ack") != std::string::npos) {
        std::lock_guard<std::mutex> lock(ack_lock_);
        ack_ = true;
    } else if (msg.find("start") != std::string::npos) {
        std::lock_guard<std::mutex> lock(ack_lock_);
        ack_frontend_ = true;
        if (lap_) lap_->unpause();
    } else if (msg.find("pause") != std::string::npos) {
        if (lap_) lap_->pause();
    }
}

json WebGUI::update_gui()
{
    json payload;
    payload["lap"] = lap_ ? lap_->check_threshold() : "";
    
    if (map_) {
        try {
            payload["map"] = map_->get_json_data().dump();
        } catch (const std::exception& e) {
            std::cerr << "[WebGUI] Error serializando JSON: " << e.what() << std::endl;
            payload["map"] = "{}";
        }
    } else {
        payload["map"] = "{}";
    }

    return payload;
}

void WebGUI::target_reached_callback(std_msgs::msg::Bool::UniquePtr msg) {
    if (msg->data && current_target_obj_ && map_) {
        current_target_obj_->setReached(true);
        current_target_obj_ = map_->getNextTarget();
        publish_current_target();
    }
}

void WebGUI::publish_current_target() {
    if (current_target_obj_) {
        geometry_msgs::msg::Point m;
        m.x = current_target_obj_->getPose().x;
        m.y = current_target_obj_->getPose().y;
        m.z = 0.0;
        pub_current_target_->publish(m);
    }
}

void WebGUI::showForces(const std::vector<double>& v1, const std::vector<double>& v2, const std::vector<double>& v3) {
    if (instance_ && instance_->map_) { instance_->map_->setCar(v1[0], v1[1]); instance_->map_->setObs(v2[0], v2[1]); instance_->map_->setAvg(v3[0], v3[1]); }
}

void WebGUI::showLocalTarget(const std::vector<double>& v) {
    if (instance_ && instance_->map_) instance_->map_->setTargetPos(v[0], v[1]);
}

std::shared_ptr<Target> WebGUI::getNextTarget() {
    if (instance_ && instance_->map_) { instance_->current_target_obj_ = instance_->map_->getNextTarget(); instance_->publish_current_target(); return instance_->current_target_obj_; }
    return nullptr;
}

void WebGUI::setTargetx(double x) { if (instance_ && instance_->map_) instance_->map_->targetx = x; }
void WebGUI::setTargety(double y) { if (instance_ && instance_->map_) instance_->map_->targety = y; }