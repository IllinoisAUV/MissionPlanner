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
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <sstream>
#include <vector>

#include "vision.h"

const float speed = 0.2;
const float kP_yaw = 0.0002;
const float kP_height = 0.0003;

class MissionController {
    public:
        MissionController();
        void Iterate();
        typedef enum { ESTOP, FORWARD, STOP, UP, DOWN, GATE, BUOY, WIRE, VISION } State;

        std::string StateString(State state) {
            switch(state) {
                case ESTOP:
                    return "estop";
                case FORWARD:
                    return "forward";
                case STOP:
                    return "stop";
                case UP:
                    return "up";
                case DOWN:
                    return "down";
                case GATE:
                    return "gate";
                case BUOY:
                    return "buoy";
                case WIRE:
                    return "wire";
                case VISION:
                    return "vision";
                default:
                    return "unknown";
            }
        }

        Vision vision_;
    private:
        void disarm();
        void arm();
        void SetLinear(float xdot, float ydot, float zdot);
        void SetAngular(float roll, float pitch, float yaw);

        void nextState();
        void killCallback(const std_msgs::Bool::ConstPtr &kill);

        void visionDoneCallback(const std_msgs::Empty::ConstPtr &done);
        void visionTargetCallback(const std_msgs::Float32MultiArray::ConstPtr &target);

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

        float target_radius_;
        float target_angle_;
};

/* static const MissionController::State mission[] = { MissionController::VISION }; */
/* static const float durations[] = { -1 }; */
static const MissionController::State mission[] = { MissionController::FORWARD };
static const float durations[] = { -1 };
const MissionController::State *MissionController::mission_ = mission;
const float *MissionController::durations_ = durations;

MissionController::MissionController(): armed_(false), estop_(true), index_(0), last_time_(0) {
    kill_sub_ =
        nh.subscribe("/kill_switch", 1, &MissionController::killCallback, this);

    angular_pub_ =
      nh.advertise<geometry_msgs::Vector3>("/vehicle/angular/setpoint", 1);
    linear_pub_ =
        nh.advertise<geometry_msgs::Vector3>("/vehicle/linear/setpoint", 1);
    arming_pub_ = nh.advertise<std_msgs::Bool>("/vehicle/arming", 2);

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
        ROS_INFO("Resetting");
        estop_ = false;
        index_ = 0;
        state_passed_ = ros::Duration(0);
        arm();
    }
    last_time_ = ros::Time::now();
}
void MissionController::visionDoneCallback(const std_msgs::Empty::ConstPtr &done) {
    nextState();
}
void MissionController::visionTargetCallback(const std_msgs::Float32MultiArray::ConstPtr &target) {
    if(target->data.size() != 2) {
        return;
    }

    target_angle_ = target->data[0];
    target_radius_ = target->data[1];
}

void MissionController::nextState() {
    index_++;
    state_passed_ = ros::Duration(0);
    ROS_INFO("New state: %s", StateString(mission_[index_]).c_str());
}

void MissionController::Iterate() {
    if(estop_) {
        return;
    }
    switch(mission_[index_]) {
        case FORWARD:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.20,0.0,0.0);
            break;
        case STOP:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.0,0.0,0.0);
            break;
        case UP:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.0,0.0,0.1);
            break;
        case DOWN:
            SetAngular(0.0,0.0,0.0);
            SetLinear(0.0,0.0,-0.1);
            break;
        case BUOY:
            // fallthrough
        case GATE:
            // fallthrough
        case WIRE:
            // fallthrough
        case VISION:
            {
                targets_data targets = vision_.findTargets();
                float angle = -1;
                float radius = -1;
                float area = -1;
                if (targets.vgate_angle != -1) {
                    angle = targets.vgate_angle;
                    radius = targets.vgate_radius;
                    area = targets.vgate_area;
                } else if (targets.buoy_angle != -1) {
                    angle = targets.buoy_angle;
                    radius = targets.buoy_radius;
                    area = targets.buoy_area;
                } else if (targets.wire_angle != -1) {
                    angle = targets.wire_angle;
                    radius = targets.wire_radius;
                    area = targets.wire_area;
                }
                if (angle == -1) {
                    SetAngular(0.0,0.0,0.0);
                    SetLinear(speed,0.0,0.0);
                    break;
                }
                // Basic P feedback control
                float y_diff = radius * cos(angle);
                float z_diff = radius * sin(angle);
                float yaw = kP_yaw * area * y_diff;
                float zdot = kP_height * area * z_diff;
                SetAngular(0.0,0.0,yaw);
                SetLinear(speed,0.0,zdot);
            }
            break;
        default:
            break;
    }
    if(last_time_ != ros::Time(0)) {
        state_passed_ += ros::Time::now() - last_time_;
    }
    last_time_ = ros::Time::now();
    if (durations_[index_] >= 0.0 && state_passed_ > ros::Duration(durations_[index_])) {
        nextState();
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission");

  ros::NodeHandle nh;
  ros::Rate r(20);

  MissionController control;
  int i = 0;
  while(ros::ok()) {
      ros::spinOnce();
      /* control.vision_.getImage(); */
      if(i % 2 == 0) {
          control.Iterate();
      }
      r.sleep();
      i++;
  }
  ros::shutdown();
  return 0;
}
