#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <angles/angles.h>

geometry_msgs::Pose2D current_pose;

enum State { FORWARD, TURN_LEFT, TURN_RIGHT, GO_BACK };
State state = FORWARD;

float wall_dist = 0.5;
float tolerance = 0.05;
float segurity_dist = 0.4;
float go_back_dist = 0.2;
float turn_dist = 0.2;

ros::Duration elapsed_turn_time;
ros::Time last_update_time;

bool closeToRight = false;
bool closeToLeft = false;
bool noWallRight = false;
bool noWallLeft = false;

std::string direccionGiro = "";

float goForward10CM = 0.0;
bool turn90Degree = false;
bool turn180Degree = false;
double initial_yaw = 0.0;

nav_msgs::Odometry current_odom;
ros::Publisher movement_pub;
ros::Time turn_start_time;
enum TurnRightSubState { WAIT_BEFORE_TURN, DO_TURN, GO_FORWARD_AFTER_TURN };
TurnRightSubState turn_right_substate = WAIT_BEFORE_TURN;
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
	current_odom = *msg;
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

    regions["rightFront"] = min(ranges, 270, 70);
    regions["front"] = min(min(ranges, 30, 30), min(ranges, 329, 30));
    regions["leftFront"] = min(ranges, 90, 70);
    regions["right"] = min(ranges, 270, 15);
    regions["left"] = min(ranges, 90, 15);
    regions["rightTurn"] = avg(ranges, 270, 15);
    regions["leftTurn"] = avg(ranges, 90, 15);
    regions["frontTurn"] = avg(avg(ranges, 30, 30), avg(ranges, 329, 30));



    if (!noWallRight && (regions["right"] >= 0.95) && state == FORWARD) noWallRight = true;
    if (!noWallLeft && (regions["left"] >= 0.95) && state == FORWARD) noWallLeft = true;

    ROS_INFO("noWallLeft:%d, noWallRight:%d", noWallLeft, noWallRight);
}

void follow_wall() {
    geometry_msgs::Twist move;
    move.linear.x = 0;
    move.angular.z = 0;  
        
    switch (state) {
	    case FORWARD:  

		if (noWallRight) {
		    state = TURN_RIGHT;
		    turn90Degree = true;
		    direccionGiro = "";
		    break;
		}

		else if (noWallLeft && regions["frontTurn"] < segurity_dist) {
		    state = TURN_LEFT;
		    turn90Degree = true;
		    direccionGiro = "";
		    break;
		} 


		if ((regions["rightFront"] > wall_dist + tolerance) && !noWallRight && direccionGiro != "derecha") {

		    //move.linear.x = 0.03;
		    /*if ((regions["front"] < segurity_dist && regions["front"] != 0.0) && !(regions["leftFront"] < wall_dist + tolerance)) {
		        move.angular.z = 0.5236;
		    }
		    else {*/
		        move.angular.z = -0.3;
		        if (regions["front"] > segurity_dist) {
						direccionGiro = "derecha";
					}
		    //}

		    //closeToRight = !closeToRight;

		    ROS_INFO("acercarse derecha, moveAngular:%f, direccionGiro: %s", move.angular.z, direccionGiro);

		}
		else if ((regions["leftFront"] > wall_dist + tolerance) && !noWallLeft && direccionGiro != "izquierda") {

		    //move.linear.x = 0.03;
		    /*if ((regions["front"] < segurity_dist && regions["front"] != 0.0) && !(regions["rightFront"] < wall_dist + tolerance)) {
		        move.angular.z = -0.5236; 
		    }
		    else {*/
		        move.angular.z = 0.3;
		        if (regions["front"] > segurity_dist) {
						direccionGiro = "izquierda";
					}
		    //}

		    //closeToLeft = !closeToLeft;

		    ROS_INFO("acercarse izquierda, moveAngular:%f, direccionGiro: %s", move.angular.z, direccionGiro);

		}
		else {
				if (regions["front"] < segurity_dist) {
					direccionGiro = "";
					/*if (!noWallLeft && !noWallRight) {
						state = GO_BACK;
						turn180Degree = true;
					}*/
				}
				else {
					move.angular.z = 0.0;
					move.linear.x = 0.15;
				}
		    //direccionGiro = "forward";
		    ROS_INFO("forward else");
		}


		ROS_INFO("front:%f, rightFront:%f, leftFront:%f, right:%f, left:%f, direccionGiro: %s, leftTurn:%f, rightTurn:%f", regions["front"], regions["rightFront"], regions["leftFront"], regions["right"], regions["left"], direccionGiro.c_str(), regions["leftTurn"], regions["rightTurn"]);

		break;

	    case TURN_LEFT:
	      	{
		static ros::Time turn_start_time;
		static enum { WAIT_BEFORE_TURN, DO_TURN, DETECT_WALLS, GO_FORWARD_AFTER_TURN } turn_left_substate = WAIT_BEFORE_TURN;
		
		
		switch (turn_left_substate) {

		case WAIT_BEFORE_TURN:
			move.linear.x = 0.0;
			move.angular.z = 0.0;
			turn_start_time = ros::Time::now();
			turn_left_substate = DO_TURN;
			ROS_INFO("Waiting 1 second before turn...");
			elapsed_turn_time = ros::Duration(0.0);
			last_update_time = ros::Time::now();
			break;

		case DO_TURN:{
			ros::Time now = ros::Time::now();
			ros::Duration dt = now - last_update_time;
			last_update_time = now;
			//if ((ros::Time::now() - turn_start_time).toSec() < 1.0) {
			/*if ((elapsed_turn_time.toSec() < 2.0)) {
				if (regions["front"] > turn_dist) {
					move.linear.x = 0.1;
					direccionGiro = "";
					elapsed_turn_time += dt; 
				}
				else {
					move.linear.x = 0.0;
					if ((regions["rightFront"] > wall_dist + tolerance) && direccionGiro != "derecha") {

						move.angular.z = -0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "derecha";
								}

					}
					else if ((regions["leftFront"] > wall_dist + tolerance) && direccionGiro != "izquierda") {
			
						move.angular.z = 0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "izquierda";
								}
					}
				}	
				ROS_INFO("elapseTime:%f", elapsed_turn_time.toSec());
				break;
			}*/

			if (turn90Degree) {
				initial_yaw = tf::getYaw(current_odom.pose.pose.orientation);
				turn90Degree = false;
			}

			double current_yaw = tf::getYaw(current_odom.pose.pose.orientation);
			double delta_yaw = angles::shortest_angular_distance(initial_yaw, current_yaw);

			if (fabs(delta_yaw) < M_PI / 2) {
				move.angular.z = 0.4;
				move.linear.x = 0.0;
			}
			else {
				move.angular.z = 0.0;
				goForward10CM = 0.0;
				turn_left_substate = DETECT_WALLS;
				turn_start_time = ros::Time::now();  // reuse timer for next phase
				ROS_INFO("Finished turning. Start going forward...");
			}
			break;
	
		}

		case DETECT_WALLS:
			/*if ((ros::Time::now() - turn_start_time).toSec() < 3.0) {
				move.linear.x = 0.1;
				move.angular.z = 0.0;0
			}*/
			{
			if ((regions["leftTurn"] >= wall_dist + 0.25 || regions["rightTurn"] >= wall_dist + 0.25) && regions["front"] > segurity_dist) {
				move.linear.x = 0.15;
				move.angular.z = 0.0;
			
				
			} else {
				if (regions["front"] <= segurity_dist) {
					goForward10CM = 0.0;
					turn90Degree = false;
					state = FORWARD;
					noWallLeft = false;
					turn_left_substate = WAIT_BEFORE_TURN;
				}
				
				else {
					turn_left_substate = GO_FORWARD_AFTER_TURN;
					move.angular.z = 0.0;
					move.linear.x = 0.0;
					elapsed_turn_time = ros::Duration(0.0);
					last_update_time = ros::Time::now();
				}
			}
			
			break;
		}
			
		case GO_FORWARD_AFTER_TURN:
			{
			ros::Time now = ros::Time::now();
			ros::Duration dt = now - last_update_time;
			last_update_time = now;
			//if ((ros::Time::now() - turn_start_time).toSec() < 1.0) {
			if ((elapsed_turn_time.toSec() < 1.5)) {
				if (regions["front"] > turn_dist) {
					move.linear.x = 0.1;
					direccionGiro = "";
					elapsed_turn_time += dt; 
				}
				else {
					move.linear.x = 0.0;
					if ((regions["rightFront"] > wall_dist + tolerance) && direccionGiro != "derecha") {

						move.angular.z = -0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "derecha";
								}

					}
					else if ((regions["leftFront"] > wall_dist + tolerance) && direccionGiro != "izquierda") {
			
						move.angular.z = 0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "izquierda";
								}
					}
				}	
				ROS_INFO("elapseTime:%f", elapsed_turn_time.toSec());			
				break;
			}
			else {
				goForward10CM = 0.0;
				turn90Degree = false;
				state = FORWARD;
				noWallLeft = false;
				turn_left_substate = WAIT_BEFORE_TURN;  // reset for next turn
				ROS_INFO("Done with forward phase after turning.");
			}
			break;
		}
		}
		ROS_INFO("left:%f, right:%f", regions["left"], regions["right"]);  
		ROS_INFO("turn left, state: %d", turn_left_substate);
		break;
		}
		
	case TURN_RIGHT: {

		static ros::Time turn_start_time;
		static enum { WAIT_BEFORE_TURN, DO_TURN, DETECT_WALLS, GO_FORWARD_AFTER_TURN } turn_right_substate = WAIT_BEFORE_TURN;
		
		
		switch (turn_right_substate) {

		case WAIT_BEFORE_TURN:
			move.linear.x = 0.0;
			move.angular.z = 0.0;
			turn_start_time = ros::Time::now();
			turn_right_substate = DO_TURN;
			ROS_INFO("Waiting 1 second before turn...");
			elapsed_turn_time = ros::Duration(0.0);
			last_update_time = ros::Time::now();
			break;

		case DO_TURN:{
			ros::Time now = ros::Time::now();
			ros::Duration dt = now - last_update_time;
			last_update_time = now;
			//if ((ros::Time::now() - turn_start_time).toSec() < 1.0) {
			if ((elapsed_turn_time.toSec() < 1.5)) {
				if (regions["front"] > turn_dist) {
					move.linear.x = 0.1;
					direccionGiro = "";
					elapsed_turn_time += dt; 
				}
				else {
					move.linear.x = 0.0;
					if ((regions["rightFront"] > wall_dist + tolerance) && direccionGiro != "derecha") {

						move.angular.z = -0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "derecha";
								}

					}
					else if ((regions["leftFront"] > wall_dist + tolerance) && direccionGiro != "izquierda") {
			
						move.angular.z = 0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "izquierda";
								}
					}
				}	
				ROS_INFO("elapseTime:%f", elapsed_turn_time.toSec());			
				break;
			}

			if (turn90Degree) {
				initial_yaw = tf::getYaw(current_odom.pose.pose.orientation);
				turn90Degree = false;
			}

			double current_yaw = tf::getYaw(current_odom.pose.pose.orientation);
			double delta_yaw = angles::shortest_angular_distance(initial_yaw, current_yaw);

			if (fabs(delta_yaw) < M_PI / 2) {
				move.angular.z = -0.4;
				move.linear.x = 0.0;
			} else {
				move.angular.z = 0.0;
				goForward10CM = 0.0;
				turn_right_substate = DETECT_WALLS;
				turn_start_time = ros::Time::now();  // reuse timer for next phase
				ROS_INFO("Finished turning. Start going forward...");
			}
			break;
	
		}

		case DETECT_WALLS:
			{/*if ((ros::Time::now() - turn_start_time).toSec() < 3.0) {
				move.linear.x = 0.1;
				move.angular.z = 0.0;
			}*/ 
			if ((regions["rightTurn"] >= wall_dist + 0.25 || regions["leftTurn"] >= wall_dist + 0.25) && regions["front"] > segurity_dist) {
				move.linear.x = 0.15;
				move.angular.z = 0.0;
			}else {
				if (regions["front"] <= segurity_dist) {
					goForward10CM = 0.0;
					turn90Degree = false;
					state = FORWARD;
					noWallRight = false;
					turn_right_substate = WAIT_BEFORE_TURN;
				}
				
				else {
					turn_right_substate = GO_FORWARD_AFTER_TURN;
					move.angular.z = 0.0;
					move.linear.x = 0.0;
					elapsed_turn_time = ros::Duration(0.0);
					last_update_time = ros::Time::now();
				}
			}
			ROS_INFO("left:%f, right:%f", regions["left"], regions["right"]);  
			break;
		}
			
		case GO_FORWARD_AFTER_TURN:
		{
			ros::Time now = ros::Time::now();
			ros::Duration dt = now - last_update_time;
			last_update_time = now;
			//if ((ros::Time::now() - turn_start_time).toSec() < 1.0) {
			if ((elapsed_turn_time.toSec() < 1.5)) {
				if (regions["front"] > turn_dist) {
					move.linear.x = 0.1;
					direccionGiro = "";
					elapsed_turn_time += dt; 
				}
				else {
					move.linear.x = 0.0;
					if ((regions["rightFront"] > wall_dist + tolerance) && direccionGiro != "derecha") {

						move.angular.z = -0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "derecha";
								}

					}
					else if ((regions["leftFront"] > wall_dist + tolerance) && direccionGiro != "izquierda") {
			
						move.angular.z = 0.3;
						if (regions["front"] > segurity_dist) {
									direccionGiro = "izquierda";
								}
					}
				}	
				ROS_INFO("elapseTime:%f", elapsed_turn_time.toSec());			
				break;
			}
			else {
				goForward10CM = 0.0;
				turn90Degree = false;
				state = FORWARD;
				noWallRight = false;
				turn_right_substate = WAIT_BEFORE_TURN;  // reset for next turn
				ROS_INFO("Done with forward phase after turning.");
			}
			break;
		}
		}
		
		

		ROS_INFO("turn right, state: %d", turn_right_substate);
		break;
	}

    
    	case GO_BACK:
    
		if (turn180Degree) {
			initial_yaw = tf::getYaw(current_odom.pose.pose.orientation);
			turn180Degree = false;
		}

		double current_yaw = tf::getYaw(current_odom.pose.pose.orientation);
		double delta_yaw = angles::shortest_angular_distance(initial_yaw, current_yaw);

		if (fabs(delta_yaw) < M_PI) {
			move.angular.z = -0.5236;
		} else {
			move.angular.z = 0.0;
			state = FORWARD;
			//ROS_INFO("Finished turning. Start going forward...");
		}
		ROS_INFO("GO_BACK");
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
