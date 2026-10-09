//
// Created by David Glogowski on 09/10/2026.
//

#include "robot_hardware.hpp"
#include "config.hpp"

using namespace webots;

static Motor* initializeMotor(Robot *robot, const std::string& motorName) {
    Motor *motor = robot->getMotor(motorName);
    motor->setVelocity(0.0);
    motor->setPosition(INFINITY);
    return motor;
}

static PositionSensor* initializePositionSensor(Robot *robot, const std::string& encoderName) {
    PositionSensor *enc = robot->getPositionSensor(encoderName);
    enc->enable(TIME_STEP);
    return enc;
}

static DistanceSensor* initializeDistanceSensor(Robot *robot, const std::string& dsName) {
    DistanceSensor *ds = robot->getDistanceSensor(dsName);
    ds->enable(TIME_STEP);
    return ds;
}

static InertialUnit* initializeInertialUnit(Robot* robot, const std::string& name) {
    InertialUnit* imu = robot->getInertialUnit(name);
    imu->enable(TIME_STEP);
    return imu;
}

Robot* initializeRobot() {
    Robot* robot = new Robot();
    robot->step(TIME_STEP);
    return robot;
}

RobotMotors initializeMotors(Robot* robot) {
    return RobotMotors {
        .leftFront  = initializeMotor(robot, "motor_4"),
        .leftBack   = initializeMotor(robot, "motor_1"),
        .rightFront = initializeMotor(robot, "motor_3"),
        .rightBack  = initializeMotor(robot, "motor_2")
    };
}

RobotSensors initializeSensors(Robot* robot) {
    return RobotSensors {
        .front = initializeDistanceSensor(robot, "ds_front"),
        .left  = initializeDistanceSensor(robot, "ds_left"),
        .right = initializeDistanceSensor(robot, "ds_right"),

        .encLF = initializePositionSensor(robot, "encoder_4"),
        .encLB = initializePositionSensor(robot, "encoder_1"),
        .encRF = initializePositionSensor(robot, "encoder_3"),
        .encRB = initializePositionSensor(robot, "encoder_2"),

        .imu   = initializeInertialUnit(robot, "imu")
    };
}

