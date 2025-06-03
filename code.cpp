#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <algorithm>

ros::Time state_timer;
double time_elapsed = 0.0;

geometry_msgs::Pose2D current_pose;
geometry_msgs::Pose2D prev_pose;
float prev_right_distance = 0.0;

ros::Time correction_timer;
double correction_interval = 1.0;

enum State { FORWARD, TURN_LEFT, TURN_RIGHT };

State state = FORWARD;

float wall_dist = 0.2;
float tolerance = 0.05;

ros::Publisher movement_pub;

std::map<std::string, float> regions;

float min(float a, float b) { return (a < b) ? a : b; }

float min(const std::vector<float>& range, int mid, int offset) {
    float min_value = std::numeric_limits<float>::infinity();
    for (int i = mid - offset; i <= mid + offset; ++i) {
        if (std::isfinite(range[i]) and range[i] != 0) {
            min_value = min(min_value, range[i]);
        }
    }
    return min_value;
}

float avg(float a, float b) { return (a + b) / 2; }

float avg(const std::vector<float>& range, int mid, int offset) {
    float sum = 0.0;
    int count = 0;
    for (int i = mid - offset; i <= mid + offset; ++i) {
        if (std::isfinite(range[i]) and range[i] != 0) {
            sum += range[i];
            ++count;
        }
    }
    return sum / count;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    const std::vector<float>& ranges = msg->ranges;
    regions["right"] = min(ranges, 270, 70);
    regions["left"] = min(ranges, 90, 70);
    regions["front"] = min(min(ranges, 15, 15), min(ranges, 345, 15));
    ROS_INFO("front:%f, right:%f, left:%f", regions["front"], regions["right"], regions["left"]);
}

float computeCorrectionAngle() {
    float dx = current_pose.x - prev_pose.x;
    float dy = current_pose.y - prev_pose.y;
    float heading_angle = atan2(dy, dx);
    float delta_dist = regions["right"] - prev_right_distance;
    float wall_angle = heading_angle - atan2(delta_dist, sqrt(dx * dx + dy * dy));
    float desired_angle = current_pose.theta;
    float angle_error = wall_angle - desired_angle;
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;
    return angle_error;
}

void follow_wall() {
    geometry_msgs::Twist move;
    move.linear.x = 0.15;
    double time_since_correction = (ros::Time::now() - correction_timer).toSec();
    if (time_since_correction >= correction_interval) {
        float angle_correction = computeCorrectionAngle();
        move.angular.z = angle_correction;
        prev_pose = current_pose;
        prev_right_distance = regions["right"];
        correction_timer = ros::Time::now();
    } else {
        move.angular.z = 0.0;
    }
    movement_pub.publish(move);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle n;
    ros::Subscriber sub_odom = n.subscribe("odom", 100, odomCallback);
    ros::Subscriber sub_scan = n.subscribe("scan", 100, scanCallback);
    movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate rate(20);
    state_timer = ros::Time::now();
    correction_timer = ros::Time::now();
    while (ros::ok()) {
        follow_wall();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
