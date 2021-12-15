/**
 * Robotics control in a simulated environment
 * Robotics Engineering - Simone Contorno
 * 
 */
// roslaunch final_assignment simulation_gmapping.launch 

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
//#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
//#include "std_srvs/SetBool.h"
//#include "std_srvs/Empty.h"
//#include "actionlib/client/simple_action_client.h"
#include <csignal>
#include <iostream>
#include <string>
#include "time.h"
// Publishers
ros::Publisher pub_goal;
ros::Publisher pub_canc;
ros::Publisher pub_vel;

// Service clients
ros::ServiceClient go_to_point;
ros::ServiceClient keyboard;
ros::ServiceClient both;

//actionlib::SimpleActionClient<actionlib_msgs::GoalID> cg("cancel_goal", true);

// Global variables

float lin_vel = 0.0;
float ang_vel = 0.0;
int flag = 0;
int drive_flag = 0;
int goal_counter = 0;
char key;
std::string id;

void manualControl() {
    //system("clear");
    printf("\n--- Manual Control ---\n\n"
    "Linear velocity advised: 0.5\n"
    "Angular velocity advised: 1.0\n");

    geometry_msgs::Twist robot_vel;
    int count = 10;
    key = 'e';
    while (key != 'q') {
        if (count % 10 == 0) 
            printf("\nCommands:\n"
            "w - Go on\n"
            "a - Turn left\n"
            "s - Go back\n"
            "d - Turn right\n"
            "--------------\n"
            "z - Increase linear velocity\n"
            "x - Decrease linear velocity\n"
            "c - Increase angular velocity\n"
            "v - Decrease angular velocity\n"
            "-----------------------------\n"
            "e - Emergency stop\n"
            "q - Quit\n");
        fflush(stdout);
        if (flag == 0) 
            printf("h - Enable drive assistance\n");
        else if (flag == 1)
            printf("h - Disable drive assistance\n");
        fflush(stdout);
        printf("\nCommand: ");
        fflush(stdout);
        std::cin >> key;

        if (key == 'w') { // Go on
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'a') { // Turn left
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = +ang_vel; 
        }
        else if (key == 's') { // Go back
            robot_vel.linear.x = -lin_vel;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'd') { // Turn right
            robot_vel.linear.x = lin_vel;
            robot_vel.angular.z = -ang_vel; 
        }
        else if (key == 'z') { // Increase linear velocity
            lin_vel += 0.1;
        }
        else if (key == 'x') { // Decrease linear velocity
            lin_vel -= 0.1;
        }
        else if (key == 'c') { // Increase angular velocity
            ang_vel += 0.1;
        }
        else if (key == 'v') { // Decrease angulare velocity
            ang_vel -= 0.1;
        }
        else if (key == 'e') { // Emergency stop
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0; 
        }
        else if (key == 'h') {
            if (flag == 0) {
                drive_flag = 1;
                flag = 1;
            }
            else if (flag == 1) {
                drive_flag = 0;
                flag = 0;
            }
        }
        else if (key == 'q') { // Quit
            robot_vel.linear.x = 0;
            robot_vel.angular.z = 0; 
            pub_vel.publish(robot_vel);
            break;
        }
        printf("Linear velocity: %f\n"
        "Angular velocity: %f\n", lin_vel, ang_vel);
        fflush(stdout);
        pub_vel.publish(robot_vel);
        count++;
    }
}

void manualControlAssisted(const sensor_msgs::LaserScan::ConstPtr& msg) {
    float mid_value = 30.0;
    geometry_msgs::Twist robot_vel;

    for (int i = 0; i < 720; i++) {
        //printf("RANGE: %f\n", msg->ranges[i]);
        if (msg->ranges[i] < mid_value)
            mid_value = msg->ranges[i];
    }
    //printf("MID VALUE: %f\n", mid_value);
    //sleep(10);
    if (mid_value < 0.5 & drive_flag == 1 & key == 'w') {
        //printf("The robot is too close to the wall! Go back!\n");
        robot_vel.linear.x = 0;
        robot_vel.angular.z = 0;
        pub_vel.publish(robot_vel);
        /*timespec time;
        time.tv_nsec = 5000000;
        nanosleep(&time, NULL);*/
    }
}

void robotCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    //printf("GOAL ID: %s\n", (msg->status.goal_id.id).c_str());
    id = msg->status.goal_id.id;
}

void controller(const sensor_msgs::LaserScan::ConstPtr& msg) {
    move_base_msgs::MoveBaseActionGoal goal_pos;
    actionlib_msgs::GoalID canc_goal;
    int x, y, in;
    // Choose the next action
    do {
        printf("\nChoose an action:\n"
            "1. Insert new coordinates to reach\n"
            "2. Cancel the current goal\n"
            "3. Manual drive\n");
        if (flag == 0) 
            printf("4. Enable drive assistance\n");
        else if (flag == 1)
            printf("4. Disable drive assistance\n");
        printf("5. Exit\n"
            "Action (type the corresponding number): ");
        std::cin >> in;
        if (in != 1 & in != 2 & in != 3 & in != 4 & in != 5)
            printf("\nERROR: type '1', '2', '3', '4' or '5'.\n");
    } while (in != 1 & in != 2 & in != 3 & in != 4 & in != 5);

    // Take coordinates to reach by the user
    if (in == 1) {
        printf("Insert coordinates to reach:\n");
        printf("X: ");
        std::cin >> x;
        printf("Y: ");
        std::cin >> y;

        // Set new coordinates to reach
        id = "goal_" + std::to_string(goal_counter);
        goal_counter++;
        goal_pos.goal_id.id = id;
        goal_pos.goal.target_pose.header.frame_id = "map";
        goal_pos.goal.target_pose.pose.orientation.w = 1;
        
        goal_pos.goal.target_pose.pose.position.x = x;
        goal_pos.goal.target_pose.pose.position.y = y;

        // Publish goal position
        pub_goal.publish(goal_pos);
    }
    
    // Cancel current goal
    else if (in == 2) {
        
        //canc_goal.id = msg->status.goal_id.id;
        canc_goal.id = id;
        pub_canc.publish(canc_goal);
        printf("Goal cancelled.\n");
    }

    // Manual drive
    else if (in == 3) {
        manualControl();
    }

    // Enable or Disable drive assistance
    else if (in == 4) {
        if (flag == 0) {
            drive_flag = 1;
            flag = 1;
        }
        else if (flag == 1) {
            drive_flag = 0;
            flag = 0;
        }
    }

    // Terminate the program
    else if (in == 5)
        exit(0);
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "final_robot");
    ros::NodeHandle nh;
    
    // Publishers
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    pub_canc = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    ros::Subscriber sub = nh.subscribe("/scan", 1000, controller);
    ros::Subscriber sub_id = nh.subscribe("/move_base/feedback", 1000, robotCallback);
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, manualControlAssisted);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

    // Set the modality
    /*
    while(true) {
        printf("\nChoose a modality:\n"
        "1. Go to the point\n"
        "2. Manual drive\n"
        "3. Assisted drive\n"
        "4. Exit\n"
        "Modality (type the corresponding number): ");
        std::cin >> in;
        if (in != 1 & in != 2 & in != 3 & in != 4)
            printf("\nERROR: type '1', '2', '3' or '4'.\n");
        else if (in == 1) 
            sub = nh.subscribe("/move_base/feedback", 1000, robotCallback);
        else if (in == 2) 
            sub = nh.subscribe("/move_base/status", 1000, manualControl);
        else if (in == 3) {
            printf("\n--- Manual Control ---\n\n"
            "Linear velocity advised: 0.5\n"
            "Angular velocity advised: 1.0\n");
            sub = nh.subscribe("/scan", 1000, manualControlAssisted);
        }
        else if (in == 4)
            break;
        ros::spin();
        flag = 0;   
    }
    */

    return 0;
}