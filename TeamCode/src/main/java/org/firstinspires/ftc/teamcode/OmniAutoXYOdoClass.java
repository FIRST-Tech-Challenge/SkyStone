package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoXYOdoClass extends LinearOpMode {

    protected ElapsedTime timer;

    public static float mmPerInch = OmniAutoXYOdoClass.MM_PER_INCH;
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

    // This function is called during loops to progress started activities
    public void performRobotActivities() {
        robot.performLifting();
        robot.performReleasing();
        robot.performStowing();
        robot.performAligning();
        robot.performGrabbing();
        robot.performStoneStacking();
	}

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
}