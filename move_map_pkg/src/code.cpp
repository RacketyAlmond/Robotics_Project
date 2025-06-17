#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <cmath>

// --- GLOBALS ---
cv::Mat map_img;
double resolution = 0.05;
double origin_x = 0.0, origin_y = 0.0;
cv::Point entrance(150, 150); // Hardcoded entrance in pixel coords (adjust later)
bool got_pose = false;
cv::Point robot_px;

// --- A* STRUCTURES ---
struct Node {
    cv::Point point;
    float f, g, h;
    Node* parent;
    Node(cv::Point p, float g, float h, Node* parent = nullptr)
        : point(p), g(g), h(h), f(g + h), parent(parent) {}
};
struct Compare { bool operator()(const Node* a, const Node* b) { return a->f > b->f; } };
bool isFree(const cv::Mat& map, int x, int y) {
    return x >= 0 && y >= 0 && x < map.cols && y < map.rows && map.at<uchar>(y, x) > 200;
}
std::vector<cv::Point> getNeighbors(const cv::Point& p) {
    return {
        {p.x+1, p.y}, {p.x-1, p.y}, {p.x, p.y+1}, {p.x, p.y-1},
        {p.x+1, p.y+1}, {p.x-1, p.y-1}, {p.x-1, p.y+1}, {p.x+1, p.y-1}
    };
}
float heuristic(const cv::Point& a, const cv::Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
std::vector<cv::Point> aStar(const cv::Mat& map, const cv::Point& start, const cv::Point& goal) {
    std::priority_queue<Node*, std::vector<Node*>, Compare> open;
    std::unordered_map<int, Node*> all_nodes;
    auto hash = [&](cv::Point pt) { return pt.y * map.cols + pt.x; };
    Node* startNode = new Node(start, 0, heuristic(start, goal));
    open.push(startNode); all_nodes[hash(start)] = startNode;
    while (!open.empty()) {
        Node* current = open.top(); open.pop();
        if (current->point == goal) {
            std::vector<cv::Point> path;
            for (Node* n = current; n != nullptr; n = n->parent)
                path.push_back(n->point);
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (cv::Point neighbor : getNeighbors(current->point)) {
            if (!isFree(map, neighbor.x, neighbor.y)) continue;
            float g = current->g + heuristic(current->point, neighbor);
            int hsh = hash(neighbor);
            if (all_nodes.count(hsh) == 0 || g < all_nodes[hsh]->g) {
                Node* next = new Node(neighbor, g, heuristic(neighbor, goal), current);
                open.push(next); all_nodes[hsh] = next;
            }
        }
    }
    std::cerr << "No path found!" << std::endl;
    return {};
}

// --- LOAD MAP ---
bool loadMap(const std::string& yaml_path) {
    YAML::Node map = YAML::LoadFile(yaml_path);
    std::string img_file = map["image"].as<std::string>();
    resolution = map["resolution"].as<double>();
    origin_x = map["origin"][0].as<double>();
    origin_y = map["origin"][1].as<double>();
    map_img = cv::imread(img_file, cv::IMREAD_GRAYSCALE);
    if (map_img.empty()) {
        std::cerr << "Failed to load map image!" << std::endl;
        return false;
    }
    std::cout << "Map loaded: " << map_img.cols << "x" << map_img.rows << std::endl;
    return true;
}

// --- ROBOT POSE CALLBACK ---
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    double rx = msg->pose.pose.position.x;
    double ry = msg->pose.pose.position.y;

    int px = (rx - origin_x) / resolution;
    int py = map_img.rows - (ry - origin_y) / resolution;

    robot_px = cv::Point(px, py);
    got_pose = true;

    std::cout << "Robot pose (px): " << robot_px << std::endl;

    std::vector<cv::Point> path = aStar(map_img, robot_px, entrance);
    if (path.empty()) return;

    cv::Mat path_map = map_img.clone();
    for (auto& p : path) path_map.at<uchar>(p) = 128;

    cv::circle(path_map, robot_px, 3, cv::Scalar(255), -1);
    cv::circle(path_map, entrance, 3, cv::Scalar(255), -1);

    cv::imwrite("/tmp/robot_path.png", path_map);
    std::cout << "Saved path to /tmp/robot_path.png" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path_planner");
    ros::NodeHandle nh;

    if (!loadMap("/home/alumne/catkin_ws/mapa.yaml")) return -1;

    ros::Subscriber sub = nh.subscribe("/amcl_pose", 1, poseCallback);
    ros::spin();

    return 0;
}
