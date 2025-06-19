#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class WallFollower {
public:
    WallFollower() {
        sub_ = nh_.subscribe("/scan", 10, &WallFollower::laserCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        geometry_msgs::Twist move;

        // Lidar distances
        float front_right_dist = getSafeRange(msg, 330);
        float right_front_dist = getSafeRange(msg, 300);
        float right_dist = getSafeRange(msg, 270);

        // Angle between right_front_dist and right_dist
        float angle = (300 - 270) * (M_PI / 180);

        // Thresholds
        float front_thresh = 0.25;
        float wall_thresh = 0.3;

        // Offset to correct errors
        float offset = 0.01;

        // Turn right if there is no wall in right (rare case)
        if (right_dist > wall_thresh) {
            ROS_INFO("right - wall_dist >>");
            move.linear.x = 0.05;
            move.angular.z = -0.22;
        }
        // Turn left if there is wall in front and right
        else if (front_right_dist < front_thresh) {
            ROS_INFO("left - front_dist <<");
            move.angular.z = 0.5;
        }
        // Turn left if getting too close to the wall
        else if (right_front_dist < right_dist/cos(angle) + offset) {
            ROS_INFO("left");
            move.linear.x = 0.05;
            move.angular.z = 0.28;
        }
        // Turn right if drifting away from the wall
        else if (right_front_dist > right_dist/cos(angle) + offset) {
            ROS_INFO("right");
            move.linear.x = 0.05;
            move.angular.z = -0.28;
        }

        pub_.publish(move);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    // Get distance from a lidar angle
    float getSafeRange(const sensor_msgs::LaserScan::ConstPtr& msg, int index) {
        float range = msg->ranges[index];

        if (std::isnan(range) || std::isinf(range) || range == 0.0) {
            return 10.0; // valor grande
        }

        return range;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follow");
    WallFollower wall_follower;
    ros::spin();
    return 0;
}
