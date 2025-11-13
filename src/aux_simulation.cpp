/*
* This file is part of mussol_simulation.
*
* Copyright (C) 2023 Antoni Tauler-Rosselló <a.tauler@uib.cat> (University of the Balearic Islands)
*
* mussol_simulation is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* mussol_simulation is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with mussol_simulation. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/UInt8.h>
#include <srv_mav_msgs/MAVVerticalState.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

// Global Variables
float last_z_vel = 0.0;
float last_height = 0.0;
nav_msgs::Odometry last_odom;

// Publishers
ros::Publisher mvs_pub;
ros::Publisher status_pub;
ros::Publisher vel_est_pub;
ros::Publisher local_odom_pub;
ros::Publisher global_odom_pub;

// Functions
nav_msgs::Odometry gazeboState2Odom(int model, gazebo_msgs::ModelStates gazebo_state){

    nav_msgs::Odometry odom;

    // Pose
    odom.pose.pose.position.x = gazebo_state.pose[model].position.x;
    odom.pose.pose.position.y = gazebo_state.pose[model].position.y;
    odom.pose.pose.position.z = gazebo_state.pose[model].position.z;

    odom.pose.pose.orientation.x = gazebo_state.pose[model].orientation.x;
    odom.pose.pose.orientation.y = gazebo_state.pose[model].orientation.y;
    odom.pose.pose.orientation.z = gazebo_state.pose[model].orientation.z;
    odom.pose.pose.orientation.w = gazebo_state.pose[model].orientation.w;

    // Velocity
    odom.twist.twist.linear.x = gazebo_state.twist[model].linear.x;
    odom.twist.twist.linear.y = gazebo_state.twist[model].linear.y;
    odom.twist.twist.linear.z = gazebo_state.twist[model].linear.z;

    odom.twist.twist.angular.x = gazebo_state.twist[model].angular.x;
    odom.twist.twist.angular.y = gazebo_state.twist[model].angular.y;
    odom.twist.twist.angular.z = gazebo_state.twist[model].angular.z;

    return odom;
}

// Callbacks
void odomGazeboClb(const gazebo_msgs::ModelStates::ConstPtr& odom)
{
    last_odom = gazeboState2Odom(1, *odom);

    // rotate twist to the robot frame
    tf::Quaternion q;
    tf::quaternionMsgToTF(last_odom.pose.pose.orientation,q);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Matrix3x3 m_rot;
    m_rot.setRPY(0, 0, -yaw);

    double velX = last_odom.twist.twist.linear.x;
    double velY = last_odom.twist.twist.linear.y;
    double velZ = last_odom.twist.twist.linear.z;

    tf::Vector3 world_vel(velX, velY, velZ);
    tf::Vector3 rot_vel = m_rot*world_vel;

    last_odom.twist.twist.linear.x = rot_vel.getX();
    last_odom.twist.twist.linear.y = rot_vel.getY();
    last_odom.twist.twist.linear.z = rot_vel.getZ();

    last_height = last_odom.pose.pose.position.z;
    last_z_vel = last_odom.twist.twist.linear.z;
}


void odomClb(const nav_msgs::Odometry::ConstPtr& odom)
{
    last_odom = *odom;

    // rotate twist to the robot frame
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,q);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Matrix3x3 m_rot;
    m_rot.setRPY(0, 0, -yaw);

    double velX = odom->twist.twist.linear.x;
    double velY = odom->twist.twist.linear.y;
    double velZ = odom->twist.twist.linear.z;

    tf::Vector3 world_vel(velX, velY, velZ);
    tf::Vector3 rot_vel = m_rot*world_vel;

    last_odom.twist.twist.linear.x = rot_vel.getX();
    last_odom.twist.twist.linear.y = rot_vel.getY();
    last_odom.twist.twist.linear.z = rot_vel.getZ();

    last_height = last_odom.pose.pose.position.z;
    last_z_vel = last_odom.twist.twist.linear.z;
}

// Timers

void statusTimer(const ros::TimerEvent& event)
{
    std_msgs::UInt8 status;
    status.data = 3;

    status_pub.publish(status);

}

void MAVVerticalStateTimer(const ros::TimerEvent& event)
{
    // if (first_imu_in || first_velocity_in || first_height_in) {
    //     return;
    // }

    srv_mav_msgs::MAVVerticalStatePtr mvs_msg(new srv_mav_msgs::MAVVerticalState);
    mvs_msg->header.stamp = ros::Time::now();
    mvs_msg->header.frame_id = "base_link";
    mvs_msg->z = last_height;
    mvs_msg->z_vel = last_z_vel;
    mvs_msg->z_acc = 0.0; // S'hauria de fer provablement
    mvs_msg->z_gnd = 0.0;

    mvs_pub.publish(mvs_msg);
}

void velEstimationTimer(const ros::TimerEvent& event)
{
    geometry_msgs::TwistWithCovarianceStampedPtr vel_est_msg(new geometry_msgs::TwistWithCovarianceStamped);
    vel_est_msg->header.stamp = ros::Time::now();
    vel_est_msg->header.frame_id = "base_link";
    vel_est_msg->twist = last_odom.twist;

    vel_est_pub.publish(vel_est_msg);
}


void odomTimer(const ros::TimerEvent& event)
{
    
    nav_msgs::Odometry local_odom;
    nav_msgs::Odometry global_odom;

    local_odom = last_odom;
    local_odom.header.stamp = ros::Time::now();
    local_odom.header.frame_id = "odom";

    local_odom_pub.publish(local_odom);

    global_odom = last_odom;
    global_odom.header.stamp = ros::Time::now();
    global_odom.header.frame_id = "map";

    global_odom_pub.publish(global_odom);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aux_simulation");
    ros::NodeHandle nh("~");

    // Parameters
    bool odom_gazebo;
    nh.param("gazebo_odom_in", odom_gazebo, false);
    ROS_INFO("Odom obtained from %s", odom_gazebo ? "Gazebo" : "GT source");

    bool publish_mvs;
    nh.param("publish_height_estimation", publish_mvs, true);

    double mvs_freq;
    nh.param("mvs_freq", mvs_freq, 50.0);
    if(publish_mvs){    
        ROS_INFO("Vertical state published at %2.2f Hz", mvs_freq);
    }

    bool publish_status;
    nh.param("publish_status", publish_status, true);

    double status_freq;
    nh.param("status_freq", status_freq, 50.0);
    if(publish_status){
        ROS_INFO("Flight status published at %2.2f Hz", status_freq);
    }

    bool publish_vel_estimation;
    nh.param("publish_velocity_estimation", publish_vel_estimation, false);

    double vel_estimation_freq;
    nh.param("mvs_freq", vel_estimation_freq, 50.0);
    if(publish_vel_estimation){    
        ROS_INFO("Velocity estimation published at %2.2f Hz", vel_estimation_freq);
    }

    double odom_freq;
    nh.param("odom_freq", odom_freq, 50.0);
    ROS_INFO("Odometry published at %2.2f Hz", odom_freq);

   

    // ROS Connections
    if(publish_mvs)mvs_pub = nh.advertise<srv_mav_msgs::MAVVerticalState>("height_out", 1);
    if(publish_status)status_pub = nh.advertise<std_msgs::UInt8>("status_out", 1);
    if(publish_vel_estimation)vel_est_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity_out", 1);
    local_odom_pub = nh.advertise<nav_msgs::Odometry>("local_odometry_out", 1);
    global_odom_pub = nh.advertise<nav_msgs::Odometry>("global_odometry_out", 1);

    ros::Subscriber odom_sub;
    if(odom_gazebo){
        odom_sub = nh.subscribe("odometry_in", 1, odomGazeboClb);
    } else {
        odom_sub = nh.subscribe("odometry_in", 1, odomClb);
    }

    ros::Timer drone_status_timer = nh.createTimer(ros::Duration(1.0 / status_freq), statusTimer);
    ros::Timer mvs_timer = nh.createTimer(ros::Duration(1.0 / mvs_freq), MAVVerticalStateTimer);
    ros::Timer vel_est_timer = nh.createTimer(ros::Duration(1.0 / vel_estimation_freq), velEstimationTimer);
    ros::Timer odom_timer = nh.createTimer(ros::Duration(1.0 / odom_freq), odomTimer);

    ros::spin();
    return 0;
}