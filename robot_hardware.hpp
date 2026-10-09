//
// Created by David Glogowski on 09/10/2026.
//

#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>

struct RobotMotors {
    webots::Motor* leftFront;
    webots::Motor* leftBack;
    webots::Motor* rightFront;
    webots::Motor* rightBack;
};

struct RobotSensors {
    webots::DistanceSensor* front;
    webots::DistanceSensor* left;
    webots::DistanceSensor* right;

    webots::PositionSensor* encLF;
    webots::PositionSensor* encLB;
    webots::PositionSensor* encRF;
    webots::PositionSensor* encRB;

    webots::InertialUnit* imu;
};

webots::Robot* initializeRobot();
RobotMotors initializeMotors(webots::Robot* robot);
RobotSensors initializeSensors(webots::Robot* robot);
