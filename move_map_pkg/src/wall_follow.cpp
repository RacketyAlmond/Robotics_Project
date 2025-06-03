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

enum State { GO_FORWARD, TURN_LEFT, TURN_RIGHT };
enum Dir { FORWARD, BACKWARD, LEFT, RIGHT };

State state = GO_FORWARD;
Dir direction = FORWARD;

float wall_dist = 0.2;
float tolerance = 0.02;

ros::Publisher movement_pub;

std::map<std::string, float> regions;

float min(float a, float b) { return (a < b) ? a : b; }

float min(const std::vector<float>& range, int mid, int offset) {
    float min_value = std::numeric_limits<float>::infinity();

    for (int i = mid-offset; i <= mid+offset; ++i) {
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

    for (int i = mid-offset; i <= mid+offset; ++i) {
        if (std::isfinite(range[i]) and range[i] != 0) {
            sum += range[i];
            ++count;
        }
    }

    return sum / count;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    current_pose.theta = yaw;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    regions["right"] = min(ranges, 270, 70);
    regions["front"] = min(min(ranges, 15, 15), min(ranges, 345, 15));

    ROS_INFO("front:%f, right:%f", regions["front"], regions["right"]);
}

float getDirectionCenter(Dir dir) {
    switch (dir) {
        case FORWARD: return 0.0;
        case BACKWARD: return M_PI;
        case LEFT: return M_PI / 2;
        case RIGHT: return -M_PI / 2;
    }
}

void changeState(State new_state) {
    state = new_state;
    state_timer = ros::Time::now();
}

void follow_wall() {
    geometry_msgs::Twist move;

    move.linear.x = 0.1;

    switch (state) {
        case GO_FORWARD:
            if (time_elapsed < 1) {

            }
            else if (regions["right"] > wall_dist + tolerance) {
                if (current_pose.theta < getDirectionCenter(direction) - 0.1) {
                    changeState(TURN_RIGHT);
                }
            }
            else if (regions["right"] < wall_dist - tolerance) {
                if (current_pose.theta > getDirectionCenter(direction) + 0.1) {
                    changeState(TURN_LEFT);
                }
            }
            break;

        case TURN_LEFT:
            if (time_elapsed < 0.5) move.angular.z = 0.4;
            else changeState(GO_FORWARD);
            break;

        case TURN_RIGHT:
            if (time_elapsed < 0.5) move.angular.z = -0.4;
            else changeState(GO_FORWARD);
            break;
    }

    //ROS_INFO("X:%f, Y:%f, theta:%f", current_pose.x, current_pose.y, current_pose.theta);

    movement_pub.publish(move);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower");

    ros::NodeHandle n;

    ros::Subscriber sub_odom = n.subscribe("odom", 100, odomCallback);
    ros::Subscriber sub_scan = n.subscribe("scan", 100, scanCallback);

    movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate rate(20);

    while (ros::ok()) {
        follow_wall();

        time_elapsed = (ros::Time::now() - state_timer).toSec();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
