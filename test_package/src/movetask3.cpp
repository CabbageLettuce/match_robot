#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace ros;

geometry_msgs::Pose p_current;
geometry_msgs::Pose p_end;
geometry_msgs::Pose p_first;

// Twist is a message that contains the linear and angular velocities of the robot
geometry_msgs::Twist twist;

Publisher p;


tf2::Quaternion q_orig, q_rot, q_new;

double yaw_end, yaw_current, junk;
int state = 0;

// Tip: Start by reading the main() function first to better understand everything
void run()
{
    ROS_INFO_STREAM("running");
    ROS_INFO("running state: %i", state);


    

    // If the current Position was received for the first time
    // the end position is set

    switch (state)
    {
        case 0:
        {
   
            p_end = p_current;
            p_first = p_current;
            p_end.position.x += 1;
            twist.linear.x = 0.5;
            ROS_INFO_STREAM("state update 1!");
            state=1;
            break;
        }

        case 1:
        {
            if (p_current.position.x < p_end.position.x)
            {
            // Set x speed to 0.5 
            twist.linear.x = 0.5;
            }

            else{
                state = 2;
                ROS_INFO_STREAM("state update 2!");
                tf2::convert(p_current.orientation , q_orig);
                double r=0, p=0, y=3.14;  // Rotate the previous pose by 180* about X
                q_rot.setRPY(r, p, y);
                q_new = q_rot*q_orig;  // Calculate the new orientation
                q_new.normalize();
            // Convert tf2 quaternion to rotation matrix
                tf2::Matrix3x3 matrix_end(q_new);
                matrix_end.getRPY(junk,junk,yaw_end);


            }
            break;
        }

        case 2:
        {
            twist.linear.x = 0;
            twist.angular.z =0.5;
            tf2::convert(p_current.orientation , q_orig);
            tf2::Matrix3x3 matrix_current(q_orig);

            // Get the current yaw_current by reference
            // We don't need roll and pitch so we reference this info to the junk double
            matrix_current.getRPY(junk, junk, yaw_current);
    



 
                if (abs(yaw_current - yaw_end) <0.1){
                    twist.angular.z = 0;
                    twist.linear.x = -0.0;
                    state = 3;
                }
            break;
        }

        case 3:
        {
            twist.linear.x = 0.5;
            if (abs(p_current.position.x - p_first.position.x)<0.05)
            {
            // Set x speed to 0.5 
            twist.linear.x = 0.0;
            }
            ROS_INFO_STREAM("finish");


        }
    }



    // Publish twist message to the cmd_vel topic of the robot
    p.publish(twist);
    ROS_INFO_STREAM("published!");
}








// This function is called when Subscriber s (line 70) read a message
void callback(const nav_msgs::Odometry msg)
{
    // Write the current pose of the robot from the message to p_current
    p_current = msg.pose.pose;

    run();
}


void callback2(const nav_msgs::Odometry msg)
{
    // Write the current pose of the robot from the message to p_current
    p_current = msg.pose.pose;

    run();
}

void callback3(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    // Write the current pose of the robot from the message to p_current
    p_current = msg.pose.pose;

    run();
}



int main(int argc, char **argv)
{
    // See WritingPublisherSubscriber Tutorial on ros.org for explanations of the following lines
    ROS_INFO("started");
    init(argc, argv, "solution_node");

    NodeHandle n;
    Rate loop_rate(5);
    int flag = 0;

    // In the following section the publisher and subscriber is set
    // To find out which one to use, enter "rostopic list" in the command line
    // Use "rostopic type <topic>" to see the data type of a topic.
    p = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);

    // switch(flag)
    // {
    
    // case 0:
    // // {
    //     Subscriber s = n.subscribe("/mobile_base_controller/odom", 1, callback);
    //     ROS_INFO_STREAM("Using odom information");
    //     // break;
    // }
    // case 1:
    // // {
    //     Subscriber s2 = n.subscribe("/ground_truth",1,callback2);
    //     ROS_INFO_STREAM("Using ground truth information");
    //     break;
    // }
    // case 2:
    // {
        Subscriber s3 = n.subscribe("/amcl_pose",1,callback3);
        ROS_INFO_STREAM("Using AMCL information");
    //     break;
    // }


 
    // }


    // Important: Don't use spinOnce() because you want a continuous Subscriber
    spin();

    return 0;
}