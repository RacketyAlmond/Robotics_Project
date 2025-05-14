#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

geometry_msgs::Pose2D current_pose;
const double linear_velocity = 0.1;  // m/s (linear velocity)
const double angular_velocity = 0.2; // rad/s (angular velocity)
const double target_distance_1m = 1.0;
const double target_distance_05m = 0.5;
const double target_distance_025m = 0.25;
double distance_travelled = 0.0;
double initial_yaw = 0.0;

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
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

void moveForward(double target_distance, ros::Publisher& movement_pub)
{
    geometry_msgs::Twist move;
    ros::Rate rate(10);
    distance_travelled = 0.0;

    ros::Time start_time = ros::Time::now();
    while (distance_travelled < target_distance) {
        move.linear.x = linear_velocity;
        movement_pub.publish(move);

        // Compute the distance travelled
        distance_travelled = linear_velocity * (ros::Time::now() - start_time).toSec();
        rate.sleep();
    }

    // Stop the robot after reaching the target distance
    move.linear.x = 0.0;
    movement_pub.publish(move);
}

void turn(double target_angle, ros::Publisher& movement_pub)
{
    geometry_msgs::Twist move;
    ros::Rate rate(10);

    double initial_yaw = current_pose.theta;
    double target_yaw = initial_yaw + target_angle;

    if (target_yaw > M_PI) target_yaw -= 2 * M_PI;  // Normalize the target angle to [-pi, pi]
    if (target_yaw < -M_PI) target_yaw += 2 * M_PI;

    ros::Time start_time = ros::Time::now();
    while (fabs(current_pose.theta - target_yaw) > 0.05) { // 0.05 rad tolerance
        move.angular.z = (target_angle > 0) ? angular_velocity : -angular_velocity;
        movement_pub.publish(move);
        rate.sleep();
    }

    // Stop the robot after completing the turn
    move.angular.z = 0.0;
    movement_pub.publish(move);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HolaBot");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("odom", 100, odomCallback);
    ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate rate(10);

    // Wait for initial pose data
    ros::spinOnce();
    initial_yaw = current_pose.theta;

    // Move 1 meter forward
    moveForward(target_distance_1m, movement_pub);
    ros::spinOnce(); // Update the robot's pose after movement

    // Turn right 90 degrees (positive angle)
    turn(M_PI / 2, movement_pub);

    // Move 0.5 meters forward
    moveForward(target_distance_05m, movement_pub);
    ros::spinOnce(); // Update the robot's pose after movement

    // Turn left 90 degrees (negative angle)
    turn(-M_PI / 2, movement_pub);

    // Move 0.25 meters forward
    moveForward(target_distance_025m, movement_pub);
    ros::spinOnce(); // Update the robot's pose after movement

    // Turn left 90 degrees again (negative angle)
    turn(-M_PI / 2, movement_pub);

    // Move 0.25 meters forward
    moveForward(target_distance_025m, movement_pub);

    ros::spinOnce();
    return 0;
}