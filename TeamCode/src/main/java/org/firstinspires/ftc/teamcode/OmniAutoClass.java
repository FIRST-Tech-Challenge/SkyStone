package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoClass extends LinearOpMode {

    protected ElapsedTime timer;

    public static float mmPerInch = OmniAutoClass.MM_PER_INCH;
    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    HardwareOmnibot robot = new HardwareOmnibot();

    // Default to 4" wheels
    private static double myWheelSize = 4.0;
    // Default to 40:1 motors
    private static double myMotorRatio = 19.2;

    // 20:1 motor = 560
    // 40:1 motor = 1120
    // 60:1 motor = 1680
    private static final double encoderClicksPerRev = 28;
    private static double clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);

    public static final float MM_PER_INCH = 25.4f;

    /**
     * @param newWheelSize  - The size of the wheels, used to calculate encoder clicks per inch
     * @param newMotorRatio - The motor gearbox ratio, used to calculate encoder clicks per inch
     */
    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        robot.resetDriveEncoders();
        robot.setInputShaping(false);
        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);
    }

    /**
     * @param position    - The current encoder position
     * @param destination - The desired encoder position
     * @param speed       - The speed of travel used to get direction
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean reachedClickPosition(int position, int destination, double speed, boolean reverseEncoders) {
        boolean result = false;

        if (reverseEncoders) {
            if (speed < 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        } else {
            if (speed > 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        }
        return result;
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        double gyroReading = robot.readIMU();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if (Math.abs(deltaAngle) > SAME_ANGLE) {
            if (deltaAngle > 0.0) {
                rotateSpeed = -rotateSpeed;
            }
        } else {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        robot.drive(xPower, yPower, rotateSpeed, 0.0, false);
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeadingForTime(double speed, double rotateSpeed, double driveAngle, double headingAngle, int driveTime, boolean stopWhenDone) {
        double endTime = timer.milliseconds() + driveTime;
        while (!isStopRequested() && (timer.milliseconds() <= endTime) && (!isStopRequested())) {
            driveAtHeading(speed, rotateSpeed, driveAngle, headingAngle);
            robot.resetReads();
        }
        if(stopWhenDone) {
            robot.setAllDriveZero();
        }
    }

    /**
     * @param speed        - The maximum speed for the robot
     * @param rotateSpeed  - The maximum rotational speed for the robot
     * @param driveAngle   - The angle to drive at
     * @param headingAngle - The angle to maintain heading for
     * @param distance     - The distance to travel
     * @param maxTime      - The time to wait before giving up
     */
    private void driveDistanceAtAngleOnHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle, double distance, int maxTime, boolean reverseEncoders) {
        //temporary fix until we understand
        double correctedDistance = distance / 1.75;

        double endTime = timer.milliseconds() + maxTime;
        int position = robot.frontLeft.getCurrentPosition();
        int finalEncoderValue;
        double gyroReading = robot.readIMU();

        if (reverseEncoders) {
            if (speed < 0) {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            }
        } else {
            if (speed < 0) {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            }
        }

        while ((!reachedClickPosition(position, finalEncoderValue, speed, reverseEncoders) && (timer.milliseconds() < endTime) && (!isStopRequested()))) {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + finalEncoderValue;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);

            // Since this is a driveForward function, the Y Axis is the only important axis
            driveAtHeading(speed, rotateSpeed, driveAngle, gyroReading);
            updateTelemetry(telemetry);

            position = robot.frontLeft.getCurrentPosition();
            robot.resetReads();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.setAllDriveZero();
    }

    /**
     * @param speed    - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime  - The time allowed before exiting without completing
     */
    public void driveDistanceForwardOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders) {
        driveDistanceAtAngleOnHeading(speed, 0.1, 90.0, robot.readIMU(), distance, maxTime, reverseEncoders);
    }

    /**
     * @param speed    - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime  - The time allowed before exiting without completing
     */
    public void driveDistanceSidewaysOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders) {
        driveDistanceAtAngleOnHeading(speed, 0.1, 0.0, robot.readIMU(), distance, maxTime, reverseEncoders);
        robot.setAllDriveZero();
    }

    /**
     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
     * @param gyroReading      - The current angle of the robot
     * @return The minumum angle to travel to get to the destination angle
     */
    private double deltaAngle(double destinationAngle, double gyroReading) {
        double result = 0.0;
        double leftResult = 0.0;
        double rightResult = 0.0;

        if (gyroReading > destinationAngle) {
            leftResult = gyroReading - destinationAngle;
            rightResult = 360.0 - gyroReading + destinationAngle;
        } else {
            leftResult = gyroReading + 360.0 - destinationAngle;
            rightResult = destinationAngle - gyroReading;
        }

        if (leftResult < rightResult) {
            result = -leftResult;
        } else {
            result = rightResult;
        }

        return result;
    }

    /**
     * @param speed       - The maximum speed to rotate the robot
     * @param targetAngle - The 0-360 degree angle to rotate the robot to
     * @param maxTime     - The time to allow before quiting
     */
    public void rotateRobotToAngle(double speed, double targetAngle, int maxTime) {
        double endTime = timer.milliseconds() + maxTime;
        double gyroReading = robot.readIMU();
        double angleRemaining = 0.0;
        final double SAME_ANGLE = 1.0;
        double rotateSpeed = 0.0;

        angleRemaining = deltaAngle(targetAngle, gyroReading);
        while ((Math.abs(angleRemaining) > SAME_ANGLE && (timer.milliseconds() < endTime) && (!isStopRequested()))) {
            telemetry.addData("Current Angle: ", gyroReading);
            telemetry.addData("Destination Angle: ", targetAngle);
            telemetry.addData("Delta Angle: ", angleRemaining);

            rotateSpeed = controlledRotationAngle(angleRemaining, speed);
            if (angleRemaining > 0.0) {
                // Positive angle, need to rotate right
                rotateSpeed = -rotateSpeed;
            }
            telemetry.addData("Rotate Speed: ", rotateSpeed);
            robot.drive(0.0, 0.0, rotateSpeed, 0.0, false);
            updateTelemetry(telemetry);

            gyroReading = robot.readIMU();
            angleRemaining = deltaAngle(targetAngle, gyroReading);
            robot.resetReads();
        }

        robot.setAllDriveZero();
    }

    /**
     * @param speed   - Max speed to rotate
     * @param angle   - Angle in degrees to rotate.  Positive right, negative left
     * @param maxTime - Timeout to quit trying
     */
    public void rotateRobot(double speed, double angle, int maxTime) {
        double gyroReading = robot.readIMU();
        double targetAngle = gyroReading + angle;

        // We won't do circles with this function, just minimum rotation.
        // Get the destination gyro angle
		targetAngle = targetAngle % 360;
		if(targetAngle < 0.0) {
			targetAngle += 360.0;
		}

        rotateRobotToAngle(speed, targetAngle, maxTime);
    }

    /**
     *
     */
    public void rotateRobotForTime(double speed, int maxTime) {
        double endTime = timer.milliseconds() + maxTime;
        while(opModeIsActive() && timer.milliseconds() < endTime && (!isStopRequested())) {
            robot.drive(0.0,0.0,speed,0.0, false);
            robot.resetReads();
        }
        robot.setAllDriveZero();
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed) {
        final double superFastDistance = 400.0;
        final double fastDistance = 200.0;
        final double mediumDistance = 100.0;
        final double fastDivider = 1.4;
        final double mediumDivider = 2.0;
        final double slowDivider = 3.0;

        double result = 0.0;

        if (distanceToTravelMm > superFastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > fastDistance) {
            result = maxSpeed / fastDivider;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationMm(double distanceToTravelMm, double maxSpeed) {
        final double fastDistance = 75.0;
        final double mediumDistance = 50.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if (distanceToTravelMm > fastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param angleToTravel - How far we are traveling in degrees
     * @param maxSpeed      - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationAngle(double angleToTravel, double maxSpeed) {
        final double fastAngle = 30.0;
        final double mediumAngle = 15.0;
        final double mediumDivider = 1.5;
        final double slowDivider = 3.0;
        double angleToTravelAbs = Math.abs(angleToTravel);

        double result = 0.0;

        if (angleToTravelAbs > fastAngle) {
            result = maxSpeed;
        } else if (angleToTravelAbs > mediumAngle) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
	 * Moved the robot a set distance from the wall, or the timeout expires.
	 * @param side       - The ToF sensor to use to distance from the wall.
	 * @param distance   - The distance in cm from the wall to move the robot.
	 * @param driveSpeed - The maximum speed to drive to position the robot,
     *                     will slow down as approaching distance.
	 * @param error      - The allowable error from desired target distance.
	 */
	public boolean distanceFromWall(HardwareOmnibot.RobotSide side, double distance, double driveSpeed, double error, int maxTime) {
		double endTime = timer.milliseconds() + maxTime;
		boolean foundDistance;
		double testDistance = 0;
		// Allow the robot to read new values from the ToF sensors.
		robot.resetReads();
		foundDistance = robot.distanceFromWall(side, distance, driveSpeed, error);
		while(timer.milliseconds() < endTime && !foundDistance && (!isStopRequested())) {
			robot.resetReads();
			foundDistance = robot.distanceFromWall(side, distance, driveSpeed, error);
		}

		return foundDistance;
	}

    /**
	 * Moved the robot a set distance from the wall, or the timeout expires.
	 * @param backLeftDistance  - The ToF sensor to use to distance from the wall.
	 * @param backLeftDistance  - The distance in cm from the foundation to move the robot.
	 * @param backRightDistance - The distance in cm from the foundation to move the robot.
	 * @param driveSpeed        - The maximum speed to drive to position the robot,
     *                            will slow down as approaching distance.
	 * @param spinSpeed         - The maximum speed to rotate the robot.
	 * @param error             - The allowable error from desired target distance.
	 */
	public boolean parallelRearTarget(double backLeftDistance, double backRightDistance, double driveSpeed, double spinSpeed, double error, int maxTime) {
		double endTime = timer.milliseconds() + maxTime;
		boolean foundDistance;
		// Allow the robot to read new values from the ToF sensors.
		robot.resetReads();
		foundDistance = robot.parallelRearTarget(backLeftDistance, backRightDistance, driveSpeed, spinSpeed, error);
		while(timer.milliseconds() < endTime && !foundDistance && (!isStopRequested())) {
			robot.resetReads();
            foundDistance = robot.parallelRearTarget(backLeftDistance, backRightDistance, driveSpeed, spinSpeed, error);
		}

		return foundDistance;
	}

	public void grabFoundation(int maxTime) {
	    double endTime = timer.milliseconds() + maxTime;
	    boolean grabbedFoundation = false;
	    robot.resetReads();
	    robot.startGrabbing();
        robot.performGrabbing();
	    while(timer.milliseconds() < endTime && robot.grabState != HardwareOmnibot.GrabFoundationActivity.IDLE && (!isStopRequested())) {
            robot.resetReads();
            robot.performGrabbing();
        }
    }

    public void moveFingers(boolean moveUp) {
		if(moveUp) {
			robot.fingersUp();
		} else {
			robot.fingersDown();
		}
		sleep(500);
	}

    public void endAuto() {
        while (!isStopRequested()) {
            sleep(100);
        }
    }

    /**
     * Rotate the robot clockwise without regard to shortest rotation
     *
     * @param maxSpeed      - Maximum speed to spin
     * @param rotationAngle - The angle to rotate the robot
     * @param maxTime       - How long to attempt to spin before giving up
     */
    public void spinClockwise(double maxSpeed, double rotationAngle, int maxTime) {
        double endTime = timer.milliseconds() + maxTime;
        double currentAngle = 0.0;
        double lastAngle = currentAngle;
        double angleTraveled = 0.0;
        double spinRate = 0.0;
        final double SAME_ANGLE = 1.0;
        final int DELTA_SLEEP = 10;
        double deltaAngle = 0.0;

        while ((timer.milliseconds() < endTime) && (Math.abs(rotationAngle - angleTraveled) > SAME_ANGLE) && (!isStopRequested())) {
            currentAngle = robot.readIMU();
            if (currentAngle >= lastAngle) {
                angleTraveled += currentAngle - lastAngle;
            } else {
                angleTraveled += 360.0 + currentAngle - lastAngle;
            }
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.addData("Last Angle: ", lastAngle);

            deltaAngle = angleTraveled - rotationAngle;
            telemetry.addData("Delta Angle: ", deltaAngle);
            spinRate = controlledRotationAngle(deltaAngle, maxSpeed);
            // We went too far, come back
            if (deltaAngle < 0.0) {
                robot.drive(0.0, 0.0, -spinRate, 0.0, false);
                telemetry.addData("Spin Rate: ", -spinRate);
            } else {
                robot.drive(0.0, 0.0, spinRate, 0.0, false);
                telemetry.addData("Spin Rate: ", spinRate);
            }
            if (isStopRequested()) {
                break;
            }
            robot.resetReads();
        }
        robot.setAllDriveZero();
    }

    /**
     * Rotate the robot without regard to shortest rotation angle
     *
     * @param maxSpeed      - Maximum speed to spin, negative is CCW
     * @param rotationAngle - The angle to rotate the robot, negative is CCW
     * @param maxTime       - How long to attempt to spin before giving up
     */
    public void spinRobot(double maxSpeed, double rotationAngle, int maxTime) {
        double endTime = timer.milliseconds() + maxTime;
        double currentAngle = robot.readIMU();
        double lastAngle = currentAngle;
        double angleTraveled = 0.0;
        double spinRate = 0.0;
        final double SAME_ANGLE = 1.0;
        final int DELTA_SLEEP = 10;
        double deltaAngle = 0.0;

        double normalizedAngle = 0.0;
        double normalizedSpeed = 0.0;

        // Final version will be speed with a positive angle value.
        // Speed will dictate CW or CCW
        if (rotationAngle < 0.0) {
            normalizedSpeed = -maxSpeed;
            normalizedAngle = -rotationAngle;
        } else {
            normalizedSpeed = maxSpeed;
            normalizedAngle = rotationAngle;
        }

        while ((timer.milliseconds() < endTime) && ((Math.abs(normalizedAngle - angleTraveled) > SAME_ANGLE) && (!isStopRequested()))
                && (!isStopRequested())) {
            currentAngle = robot.readIMU();
            if (normalizedSpeed < 0.0) {
                if (currentAngle >= lastAngle) {
                    angleTraveled += currentAngle - lastAngle;
                } else {
                    angleTraveled += 360.0 + currentAngle - lastAngle;
                }
            } else {
                if (currentAngle <= lastAngle) {
                    angleTraveled += lastAngle - currentAngle;
                } else {
                    angleTraveled += 360.0 + lastAngle - currentAngle;
                }
            }
            telemetry.addData("Current Angle: ", currentAngle);
            telemetry.addData("Last Angle: ", lastAngle);
            lastAngle = currentAngle;

            deltaAngle = normalizedAngle - angleTraveled;
            telemetry.addData("Delta Angle: ", deltaAngle);
            spinRate = controlledRotationAngle(deltaAngle, normalizedSpeed);

            if (normalizedSpeed < 0.0) {
                // CCW
                // We went too far, come back
                if (deltaAngle < 0.0) {
                    robot.drive(0.0, 0.0, -spinRate, 0.0, false);
                    telemetry.addData("Spin Rate: ", spinRate);
                } else {
                    robot.drive(0.0, 0.0, spinRate, 0.0, false);
                    telemetry.addData("Spin Rate: ", -spinRate);
                }
            } else {
                // CW
                // We went too far, come back
                if (deltaAngle < 0.0) {
                    robot.drive(0.0, 0.0, -spinRate, 0.0, false);
                    telemetry.addData("Spin Rate: ", -spinRate);
                } else {
                    robot.drive(0.0, 0.0, spinRate, 0.0, false);
                    telemetry.addData("Spin Rate: ", spinRate);
                }
            }

            telemetry.addData("Destination Angle: ", normalizedAngle);
            telemetry.addData("Traveled Angle: ", angleTraveled);
            telemetry.update();
            robot.resetReads();
        }
        robot.setAllDriveZero();
    }
}