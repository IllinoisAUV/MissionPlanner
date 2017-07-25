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

        State state_;
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
static const float durations[] = { 60.0, -1};
const float *MissionController::durations_ = durations;

MissionController::MissionController(): armed_(false), state_(ESTOP), index_(0), last_time_(0) {
    kill_sub_ =
        nh.subscribe("/vehicle/kill_switch", 1, &MissionController::killCallback, this);

    angular_pub_ =
      nh.advertise<geometry_msgs::Vector3>("/vehicle/angular/setpoint", 1);
    linear_pub_ =
        nh.advertise<geometry_msgs::Vector3>("/vehicle/linear/setpoint", 1);
    arming_pub_ =
        nh.advertise<std_msgs::Bool>("/vehicle/arming", 1);
}

void MissionController::arm() {
    std_msgs::Bool msg;
    msg.data = true;
    arming_pub_.publish(msg);
}

void MissionController::disarm() {
    std_msgs::Bool msg;
    msg.data = false;
    arming_pub_.publish(msg);
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
        state_ = ESTOP;
        disarm();
    } else {
        state_ = mission_[index_];
        if (!armed_) {
            arm();
        }
    }
    last_time_ = ros::Time::now();
}

void MissionController::Iterate() {
    switch(state_) {
        case STRAIGHT:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.10,0.0,0.0);
            break;
        case STOP:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.0,0.0,0.0);
            break;
        case ESTOP:
            break;
        default:
            break;
    }
    if(state_ != ESTOP) {
        if(last_time_ == ros::Time(0)) {
            state_passed_ += ros::Time::now() - last_time_;
        }
        last_time_ = ros::Time::now();
    }
    if (durations_[index_] >= 0.0 && state_passed_ > ros::Duration(durations_[index_])) {
        index_++;
        state_passed_ = ros::Duration(0);
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission");

  ros::NodeHandle nh;
  MissionController control;

  ros::Rate r(5);
  while(ros::ok()) {
      control.Iterate();
      ros::spinOnce();
      r.sleep();
  }
  ros::shutdown();
  return 0;
}
