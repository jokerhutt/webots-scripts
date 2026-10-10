// webots
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

// stdlib
#include <iostream>
#include <cmath>
#include <algorithm>

// own headers
#include "config.hpp"
#include "robot_hardware.hpp"

using namespace webots;

double getWallDistance(const DistanceSensor* sensor, const double offset = 0.0) {
    return sensor->getValue() + offset;
}

double getDistanceTravelled(const RobotSensors& sensors) {
    double leftRotation = sensors.encLF->getValue();
    double rightRotation = sensors.encRF->getValue();

    return (leftRotation + rightRotation) / 2.0 * WHEEL_RADIUS;
}

double calculateWallCorrection(double wallDistance) {
    double error = wallDistance - TARGET_WALL_DISTANCE;
    return WALL_KP * error;
}

void setMotorSpeeds(
    const RobotMotors& motors,
    double leftSpeed,
    double rightSpeed
) {
    motors.leftFront->setVelocity(leftSpeed);
    motors.leftBack->setVelocity(leftSpeed);

    motors.rightFront->setVelocity(rightSpeed);
    motors.rightBack->setVelocity(rightSpeed);
}

enum class RobotState {
    DRIVING,
    TURNING_RIGHT,
    TURNING_LEFT,
    PREPARING_RIGHT_TURN,
    TURNING_AROUND,
    BRAKING,
    STOPPED,
};

struct RobotContext {
    RobotState state = RobotState::DRIVING;
    RobotState nextState = RobotState::DRIVING;
    double stateStartTime = 0.0;
    double stateStartDistance = 0.0;
    double targetHeading = 0.0;
};

void updateState(Robot* robot, RobotContext* robotContext, RobotSensors sensors, double distanceTravelled) {
    double distRight = getWallDistance(sensors.right, 0.02);
    double distFront = getWallDistance(sensors.front, 0.02);
    double distLeft = getWallDistance(sensors.left, 0.02);

    switch (robotContext->state) {

        // if driving
        case RobotState::DRIVING:
            // if no wall to the right
            if (distRight > 0.4 && distanceTravelled - robotContext->stateStartDistance >= 0.2) {
                robotContext->state = RobotState::PREPARING_RIGHT_TURN;
                robotContext->stateStartDistance = distanceTravelled;
                // else if wall in front
            } else if (distFront + 0.105 <= std::min(distRight, 0.2) + 0.04) {
                robotContext->state = RobotState::BRAKING;
                // if no wall to left
                if (distLeft > 0.4) {
                    robotContext->nextState = RobotState::TURNING_LEFT;
                    // else turn around
                } else {
                    robotContext->nextState = RobotState::TURNING_AROUND;
                }
                robotContext->stateStartTime = robot->getTime();
            }
            break;

        case RobotState::BRAKING:
            // if half a second has passed
            if (robot->getTime() - robotContext->stateStartTime >= 0.5) {
                double heading = sensors.imu->getRollPitchYaw()[2];

                switch (robotContext->nextState) {
                    case RobotState::TURNING_RIGHT:
                        robotContext->targetHeading = std::round((heading - M_PI / 2.0) / (M_PI / 2.0)) * (M_PI / 2.0);
                        break;

                    case RobotState::TURNING_LEFT:
                        robotContext->targetHeading = std::round((heading + M_PI / 2.0) / (M_PI / 2.0)) * (M_PI / 2.0);
                        break;

                    case RobotState::TURNING_AROUND:
                        robotContext->targetHeading = std::round((heading + M_PI) / (M_PI / 2.0)) * (M_PI / 2.0);
                        break;

                    default:
                        break;
                }

                robotContext->state = robotContext->nextState;
            }
            break;

        case RobotState::PREPARING_RIGHT_TURN:
            if (distanceTravelled - robotContext->stateStartDistance >= 0.2) {
                robotContext->state = RobotState::BRAKING;
                robotContext->nextState = RobotState::TURNING_RIGHT;
                robotContext->stateStartTime = robot->getTime();
            }
            break;

        case RobotState::TURNING_LEFT:
        case RobotState::TURNING_RIGHT:

        case RobotState::TURNING_AROUND: {
            double currentHeading = sensors.imu->getRollPitchYaw()[2];
            double headingError = std::remainder(
                robotContext->targetHeading - currentHeading,
                2.0 * M_PI
            );

            if (std::abs(headingError) < 0.03) {
                robotContext->state = RobotState::DRIVING;
                robotContext->stateStartDistance = distanceTravelled;
            }
            break;
        }

        case RobotState::STOPPED:
            break;
        }
    }

void executeState(const RobotContext& context, const RobotSensors& sensors, const RobotMotors& motors) {
    switch (context.state) {

        case RobotState::DRIVING: {
            double heading = sensors.imu->getRollPitchYaw()[2];
            double error = std::remainder(context.targetHeading - heading, 2.0 * M_PI);

            setMotorSpeeds(motors, BASE_SPEED - 5.0 * error, BASE_SPEED + 5.0 * error);
            break;
        }

        case RobotState::BRAKING:
            setMotorSpeeds(motors, 0.0, 0.0);
            break;

        case RobotState::STOPPED:
            setMotorSpeeds(motors, 0.0, 0.0);
            break;

        case RobotState::PREPARING_RIGHT_TURN:
            setMotorSpeeds(motors, BASE_SPEED, BASE_SPEED);
            break;

        case RobotState::TURNING_RIGHT:
            setMotorSpeeds(motors, TURN_SPEED, -TURN_SPEED);
            break;

        case RobotState::TURNING_LEFT:
            setMotorSpeeds(motors, -TURN_SPEED, TURN_SPEED);
            break;

        case RobotState::TURNING_AROUND:
            setMotorSpeeds(motors, -TURN_SPEED, TURN_SPEED);
            break;
    }
}


int main(int argc, char **argv) {

    // hardware setup
    Robot* robot = initializeRobot();
    RobotMotors robotMotors = initializeMotors(robot);
    RobotSensors robotSensors = initializeSensors(robot);

    // state setup
    RobotContext robotContext;
    robotContext.targetHeading = robotSensors.imu->getRollPitchYaw()[2];

    // Main loop:
    while (robot->step(TIME_STEP) != -1) {

        double distanceTravelled = getDistanceTravelled(robotSensors);
        updateState(robot, &robotContext, robotSensors, distanceTravelled);
        executeState(robotContext, robotSensors, robotMotors);

        std::cout << (int)robotContext.state << "  F:" << robotSensors.front->getValue()
          << "  L:" << robotSensors.left->getValue()
          << "  R:" << robotSensors.right->getValue() << std::endl;

    };

    delete robot;
    return 0;
}
