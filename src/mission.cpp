/*
 * File: automation_ws/src/automation.cpp
 * Author: Shubhankar Agarwal <shubhankar0109@@gmail.com>
 * Date: February 2017
 * Description: Automation the control of the bluerov.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/StreamRate.h>
#include <std_msgs/Bool.h>

class MissionController {
    public:
        MissionController();
        void Iterate();
        typedef enum { ESTOP, STRAIGHT, STOP } State;

        std::string StateString(State state) {
            switch(state) {
                case ESTOP:
                    return "estop";
                case STRAIGHT:
                    return "straight";
                case STOP:
                    return "stop";
                default:
                    return "unknown";
            }
        }
    private:
        void disarm();
        void arm();
        void SetLinear(float xdot, float ydot, float zdot);
        void SetAngular(float roll, float pitch, float yaw);

        void killCallback(const std_msgs::Bool::ConstPtr &kill);

        ros::NodeHandle nh;

        ros::Publisher angular_pub_;
        ros::Publisher linear_pub_;
        ros::Publisher arming_pub_;

        ros::Subscriber kill_sub_;

        bool estop_;
        bool armed_;
        ros::NodeHandle nh_;
        static const State *mission_;
        static const float *durations_; // seconds
        int index_;
        ros::Duration state_passed_;
        ros::Time last_time_;
};

static const MissionController::State mission[] = { MissionController::STRAIGHT, MissionController::STOP };
const MissionController::State *MissionController::mission_ = mission;
static const float durations[] = { 5.0, -1};
const float *MissionController::durations_ = durations;

MissionController::MissionController(): armed_(false), estop_(true), index_(0), last_time_(0) {
    kill_sub_ =
        nh.subscribe("/kill_switch", 1, &MissionController::killCallback, this);

    angular_pub_ =
      nh.advertise<geometry_msgs::Vector3>("/vehicle/angular/setpoint", 1);
    linear_pub_ =
        nh.advertise<geometry_msgs::Vector3>("/vehicle/linear/setpoint", 1);
    arming_pub_ = nh.advertise<std_msgs::Bool>("/vehicle/arming", 2);
    if (!arming_pub_) {
        ROS_ERROR("Failed to advertise to /vehicle/arming");
    }

    ros::Rate rate(20);
    while(arming_pub_.getNumSubscribers() == 0) {
        ros::spinOnce();
        rate.sleep();
    }
}

void MissionController::arm() {
    ROS_INFO("Arming");
    std_msgs::Bool msg;
    msg.data = true;
    arming_pub_.publish(msg);
    armed_ = true;
}

void MissionController::disarm() {
    ROS_INFO("Disarming");
    std_msgs::Bool msg;
    msg.data = false;
    arming_pub_.publish(msg);
    armed_ = false;
}

void MissionController::SetAngular(float roll, float pitch, float yaw) {
    geometry_msgs::Vector3 msg;
    msg.x = roll;
    msg.y = pitch;
    msg.z = yaw;
    angular_pub_.publish(msg);
}


void MissionController::SetLinear(float xdot, float ydot, float zdot) {
    geometry_msgs::Vector3 msg;
    msg.x = xdot;
    msg.y = ydot;
    msg.z = zdot;
    linear_pub_.publish(msg);
}

void MissionController::killCallback(const std_msgs::Bool::ConstPtr &kill) {
    if(kill->data) {
        ROS_INFO("Disabling");
        estop_ = true;
        disarm();
    } else {
        ROS_INFO("Enabling");
        estop_ = false;
        arm();
    }
    last_time_ = ros::Time::now();
}

void MissionController::Iterate() {
    if(estop_) {
        return;
    }
    switch(mission_[index_]) {
        case STRAIGHT:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.10,0.0,0.0);
            break;
        case STOP:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.0,0.0,0.0);
            break;
        default:
            break;
    }
    if(last_time_ != ros::Time(0)) {
        state_passed_ += ros::Time::now() - last_time_;
    }
    last_time_ = ros::Time::now();
    if (durations_[index_] >= 0.0 && state_passed_ > ros::Duration(durations_[index_])) {
        index_++;
        state_passed_ = ros::Duration(0);
        ROS_INFO("New state: %s", StateString(mission_[index_]).c_str());
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission");

  ros::NodeHandle nh;
  ros::Rate r(10);

  MissionController control;
  while(ros::ok()) {
      ros::spinOnce();
      control.Iterate();
      r.sleep();
  }
  ros::shutdown();
  return 0;
}
