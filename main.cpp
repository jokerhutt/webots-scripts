#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 64
#define MAX_SPEED 10.0
#define COLLISION_DISTANCE 0.20
#define TURN_STEPS 12
#define STOP_STEPS 5

using namespace webots;

double getWallDistance(const DistanceSensor* sensor, const double offset = 0.0) {
    return sensor->getValue() + offset;
}

bool collidesWithWall(const double distance, const double maxDistance) {
    return distance < maxDistance;
}

enum class RobotDirection {
    Forward,
    TurnLeft,
    TurnRight,
    Back,
    Stop
};

RobotDirection decideDirection(
    const double distFront,
    const double collisionDistance,
    int& stopStepsLeft,
    int& turnStepsLeft
) {
    if (stopStepsLeft > 0) {
        stopStepsLeft--;
        return RobotDirection::Stop;
    }

    if (turnStepsLeft > 0) {
        turnStepsLeft--;
        return RobotDirection::TurnRight;
    }

    if (collidesWithWall(distFront, collisionDistance)) {
        stopStepsLeft = STOP_STEPS;
        turnStepsLeft = TURN_STEPS;
        return RobotDirection::Stop;
    }

    return RobotDirection::Forward;
}

void moveRobot(
    const RobotDirection direction,
    Motor* leftBackMotor,
    Motor* rightBackMotor,
    Motor* rightFrontMotor,
    Motor* leftFrontMotor,
    const double speed
) {
    double left = 0.0;
    double right = 0.0;

    switch (direction) {
        case RobotDirection::Forward:   left =  speed; right =  speed; break;
        case RobotDirection::Back:      left = -speed; right = -speed; break;
        case RobotDirection::TurnLeft:  left = -speed; right =  speed; break;
        case RobotDirection::TurnRight: left =  speed; right = -speed; break;
        case RobotDirection::Stop:      left =  0.0;   right =  0.0;   break;
    }

    leftBackMotor->setVelocity(left);
    leftFrontMotor->setVelocity(left);
    rightBackMotor->setVelocity(right);
    rightFrontMotor->setVelocity(right);
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftBackMotor = robot->getMotor("motor_1");
    Motor *rightBackMotor = robot->getMotor("motor_2");
    Motor *rightFrontMotor = robot->getMotor("motor_3");
    Motor *leftFrontMotor = robot->getMotor("motor_4");

    leftBackMotor->setPosition(INFINITY);
    rightBackMotor->setPosition(INFINITY);
    rightFrontMotor->setPosition(INFINITY);
    leftFrontMotor->setPosition(INFINITY);

    leftBackMotor->setVelocity(0.0);
    rightBackMotor->setVelocity(0.0);
    rightFrontMotor->setVelocity(0.0);
    leftFrontMotor->setVelocity(0.0);

    PositionSensor* encLB = robot->getPositionSensor("encoder_1");
    PositionSensor* encRB = robot->getPositionSensor("encoder_2");
    PositionSensor* encRF = robot->getPositionSensor("encoder_3");
    PositionSensor* encLF = robot->getPositionSensor("encoder_4");
    encLB->enable(TIME_STEP);
    encRB->enable(TIME_STEP);
    encRF->enable(TIME_STEP);
    encLF->enable(TIME_STEP);

    DistanceSensor* dsFront = robot->getDistanceSensor("ds_front");
    DistanceSensor* dsBack = robot->getDistanceSensor("ds_back");
    dsFront->enable(TIME_STEP);
    dsBack->enable(TIME_STEP);

    robot->step(TIME_STEP);

    RobotDirection robotDirection = RobotDirection::Forward;

    int stopStepsLeft = 0;
    int turnStepsLeft = 0;

    // Main loop:
    while (robot->step(TIME_STEP) != -1) {

        const double distFront = getWallDistance(dsFront, 0.02);

        std::cout << "wall front: " << distFront << " m" << std::endl;

        robotDirection = decideDirection(distFront, COLLISION_DISTANCE, stopStepsLeft, turnStepsLeft);
        moveRobot(robotDirection, leftBackMotor, rightBackMotor, rightFrontMotor, leftFrontMotor, 0.5 * MAX_SPEED);

    };

    delete robot;
    return 0;
}
