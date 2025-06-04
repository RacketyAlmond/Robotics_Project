#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <algorithm>
#include <string>

geometry_msgs::Pose2D current_pose;

enum State { FORWARD, TURN_LEFT, TURN_RIGHT };

State state = FORWARD;

float wall_dist = 0.2;
float tolerance = 0.04;
float segurity_dist = 0.15;

bool closeToRight = false;
bool closeToLeft = false;
bool noWallRight = false;
bool noWallLeft = false;
std::string direccionGiro = "";

float goForward10CM = 0.0;
float turn90Degree = 0.0;

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

    //regions["right"] = avg(ranges, 248, 292); //msg->ranges[270];
    //regions["fright"] = avg(ranges, 293, 337); //msg->ranges[315];
    //regions["front"] = avg(avg(ranges, 338, 359), avg(ranges, 0, 22)); //msg->ranges[0];
    //regions["fleft"] = avg(ranges, 23, 67); //msg->ranges[45];
    //regions["left"] = avg(ranges, 68, 112); //msg->ranges[90];

    regions["right"] = min(ranges, 270, 70);
    regions["front"] = min(min(ranges, 30, 30), min(ranges, 329, 30));
    regions["left"] = min(ranges, 90, 70);

    if (!noWallRight && (regions["right"] >= 0.35)) noWallRight = true;
    if (!noWallLeft && (regions["left"] >= 0.35)) noWallLeft = true;

    ROS_INFO("noWallLeft:%d, noWallRight:%d", noWallLeft, noWallRight);
}

void follow_wall() {
    geometry_msgs::Twist move;

    switch (state) {
    case FORWARD:
        move.linear.x = 0;
        move.angular.z = 0;    


        /* if (noWallRight) {
            state = TURN_RIGHT;
        }

        else if (noWallLeft) {
            state = TURN_LEFT;
        } */


        if ((regions["right"] > wall_dist + tolerance) && direccionGiro != "derecha") {

            //move.linear.x = 0.03;
            if ((regions["front"] < segurity_dist && regions["front"] != 0.0) && !(regions["left"] < wall_dist + tolerance)) {
                move.angular.z = 0.1309;
            }
            else {
                move.angular.z = -0.1309;
                if (regions["front"] > segurity_dist) {
					direccionGiro = "derecha";
					move.angular.z = 0.2618;
				}
            }

            //closeToRight = !closeToRight;

            ROS_INFO("acercarse derecha, moveAngular:%f, direccionGiro: %s", move.angular.z, direccionGiro);

        }
        else if ((regions["left"] > wall_dist + tolerance) && direccionGiro != "izquierda") {

            //move.linear.x = 0.03;
            if ((regions["front"] < segurity_dist && regions["front"] != 0.0) && !(regions["right"] < wall_dist + tolerance)) {
                move.angular.z = -0.1309; 
            }
            else {
                move.angular.z = 0.1309;
                if (regions["front"] > segurity_dist) {
					direccionGiro = "izquierda";
					move.angular.z = 0.2618;
				}
            }

            //closeToLeft = !closeToLeft;

            ROS_INFO("acercarse izquierda, moveAngular:%f, direccionGiro: %s", move.angular.z, direccionGiro);

        }
        else {
			if (regions["front"] < segurity_dist) {
				direccionGiro = "";
			}
			else {
				move.angular.z = 0.0;
				move.linear.x = 0.05;
			}
            //direccionGiro = "forward";
            ROS_INFO("forward else");
        }


        ROS_INFO("front:%f, right:%f, left:%f", regions["front"], regions["right"], regions["left"]);

        break;

    case TURN_LEFT:
        move.angular.z = 0;
        move.linear.x = 0;
        if (goForward10CM < 0.1) {
            move.linear.x = 0.05;
            goForward10CM += 0.05;
        }
        else if (turn90Degree < 1.5708) {
            move.angular.z = 0.5236;
            turn90Degree += 0.5236;
        }
        else {
            goForward10CM = 0.0;
            turn90Degree = 0.0;
            state = FORWARD;
            noWallLeft = false;
        }
        ROS_INFO("turn left");

        break;

    case TURN_RIGHT:
        move.angular.z = 0;
        move.linear.x = 0;
        if (goForward10CM < 0.1) {
            move.linear.x = 0.05;
            goForward10CM += 0.05;
        }
        else if (turn90Degree < 1.5708) {
            move.angular.z = -0.5236;
            turn90Degree += 0.5236;
        }
        else {
            goForward10CM = 0.0;
            turn90Degree = 0.0;
            state = FORWARD;
            noWallRight = false;
        }
        ROS_INFO("turn right");
        break;
    }

    //move.angular.z = -0.2;
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

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
