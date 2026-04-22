#include "Map.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

Target::Target(const std::string& id, const Pose3d& pose, bool active, bool reached)
    : id(id), pose(pose), reached(reached), active(active) {}

std::string Target::getId() const { return id; }
Pose3d Target::getPose() const { return pose; }
bool Target::isReached() const { return reached; }
void Target::setReached(bool value) { reached = value; }

Map::Map(std::function<LaserData()> laser_cb, std::function<Pose3d()> pose_cb)
    : carx(0), cary(0), obsx(0), obsy(0), avgx(0), avgy(0),
      targetx(0), targety(0), laser_callback_(laser_cb), pose_callback_(pose_cb) {
    
    std::ifstream file("/resources/exercises/obstacle_avoidance/simple_circuit_targets.json");
    if (!file.is_open()) file.open("./simple_circuit_targets.json");
    
    if (file.is_open()) {
        nlohmann::json j;
        try {
            file >> j;
            if (j.contains("targets")) {
                for (const auto& t : j["targets"]) {
                    Pose3d p; p.x = t["x"]; p.y = t["y"];
                    targets_.push_back(std::make_shared<Target>(t["name"], p));
                }
            }
        } catch (...) {}
    }
}

void Map::setCar(double x, double y) { 
    std::lock_guard<std::mutex> lock(data_mutex_); 
    carx = (std::isnan(x) || std::isinf(x)) ? 0.0 : x; 
    cary = (std::isnan(y) || std::isinf(y)) ? 0.0 : y; 
}

void Map::setObs(double x, double y) { 
    std::lock_guard<std::mutex> lock(data_mutex_); 
    obsx = (std::isnan(x) || std::isinf(x)) ? 0.0 : x; 
    obsy = (std::isnan(y) || std::isinf(y)) ? 0.0 : y; 
}

void Map::setAvg(double x, double y) { 
    std::lock_guard<std::mutex> lock(data_mutex_); 
    avgx = (std::isnan(x) || std::isinf(x)) ? 0.0 : x; 
    avgy = (std::isnan(y) || std::isinf(y)) ? 0.0 : y; 
}

void Map::setTargetPos(double x, double y) { 
    std::lock_guard<std::mutex> lock(data_mutex_); 
    targetx = (std::isnan(x) || std::isinf(x)) ? 0.0 : x; 
    targety = (std::isnan(y) || std::isinf(y)) ? 0.0 : y; 
}

nlohmann::json Map::get_json_data() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    payload_["pose"] = setPose(pose_callback_());
    payload_["target"] = {targetx, targety};
    payload_["car"] = {carx, cary};
    payload_["obstacle"] = {obsx, obsy};
    payload_["average"] = {avgx, avgy};
    
    auto laser_res = setLaserValues();
    payload_["laser"] = laser_res.first;
    payload_["max_range"] = laser_res.second;

    return payload_;
}

std::shared_ptr<Target> Map::getNextTarget() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& t : targets_) {
        if (!t->isReached()) {
            targetx = t->pose.x;
            targety = t->pose.y;
            return t;
        }
    }
    if (targets_.empty()) return nullptr;
    for (auto& t : targets_) t->setReached(false);
    targetx = targets_[0]->pose.x;
    targety = targets_[0]->pose.y;
    return targets_[0];
}

void Map::reset() { 
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& t : targets_) t->setReached(false); 
}

std::vector<double> Map::setPose(const Pose3d& p) { 
    return {p.x, p.y, p.yaw}; 
}

std::pair<nlohmann::json, double> Map::setLaserValues() {
    LaserData laser = laser_callback_();
    nlohmann::json points = nlohmann::json::array();
    double max_r = laser.values.empty() ? 10.0 : laser.maxRange;

    if (!laser.values.empty()) {
        double step = (laser.values.size() > 1) ? ((laser.maxAngle - laser.minAngle) / (laser.values.size() - 1)) : 0.0;
        for (size_t i = 0; i < laser.values.size(); ++i) {
            double d = laser.values[i];
            if (std::isnan(d) || std::isinf(d)) d = max_r;
            
            double a = laser.minAngle + i * step;
            if (std::isnan(a) || std::isinf(a)) a = 0.0;
            
            points.push_back({d, a});
        }
    } else {
        for (int i = 0; i < 180; ++i) points.push_back({0.0, 0.0});
    }
    return {points, max_r};
}