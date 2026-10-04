#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define TIME_STEP 64
#define MAX_SPEED 10.0

using namespace webots;


double getVelocity(
    const PositionSensor* encL,
    const PositionSensor* encR,
    double& prevL,
    double& prevR,
    const double dt,
    const double wheelRadius
) {
    const double l = encL->getValue();
    const double r = encR->getValue();

    const double wL = (l - prevL) / dt;
    const double wR = (r - prevR) / dt;

    prevL = l;
    prevR = r;

    return wheelRadius * (wL + wR) / 2;
}

double getWallDistance(const DistanceSensor* sensor, const double offset = 0.0) {
    return sensor->getValue() + offset;
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

    const double dt = TIME_STEP / 1000.0;

    robot->step(TIME_STEP);  // let encoders get a first reading
    double prevL = encLB->getValue();
    double prevR = encRB->getValue();

    bool moveForward = true;
    double velocityScalar = 1;

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(TIME_STEP) != -1) {

        const double distFront = getWallDistance(dsFront, 0.02);
        const double distBack = getWallDistance(dsBack, 0.02);

        std::cout << "wall front: " << distFront << " m" << std::endl;
        std::cout << "wall back: " << distBack << " m" << std::endl;

        if (moveForward) {
            velocityScalar = 1;
        } else {
            velocityScalar = -1;
        }

        leftBackMotor->setVelocity(velocityScalar * MAX_SPEED);
        rightBackMotor->setVelocity(velocityScalar * MAX_SPEED);
        rightFrontMotor->setVelocity(velocityScalar * MAX_SPEED);
        leftFrontMotor->setVelocity(velocityScalar * MAX_SPEED);

        if (distFront < distBack) {
            if (distFront < 0.15) {
                moveForward = false;
            }
        } else {
            if (distBack < 0.15) {
                moveForward = true;
            }
        }

        double v = getVelocity(encLB, encRB, prevL, prevR, dt, 0.0205);

        std::cout << "speed: " << v << "m/s" << std::endl;
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
