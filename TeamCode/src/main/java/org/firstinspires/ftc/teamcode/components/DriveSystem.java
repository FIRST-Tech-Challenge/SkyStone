package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.EnumMap;

public class DriveSystem {

    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public EnumMap<MotorNames, DcMotor> motors;

    public IMUSystem imuSystem;

    private final int TICKS_IN_INCH = 69;

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     */
    public DriveSystem(EnumMap<MotorNames, DcMotor> motors, BNO055IMU imu) {
        this.motors = motors;
        initMotors();
        imuSystem = new IMUSystem(imu);
    }

    /**
     * Set the power of the drive system
     * @param power power of the system
     */
    public void setMotorPower(double power) {
        for (DcMotor motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void initMotors() {

        motors.forEach((name, motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    break;
            }
        });

        setMotorPower(0);
    }

    /**
     * Clips joystick values and drives the motors.
     * @param rightX Right X joystick value
     * @param rightY Right Y joystick value
     * @param leftX Left X joystick value
     * @param leftY Left Y joystick value in case you couldn't tell from the others
     * @param slowDrive Set to true for 30 % motor power.
     */
    // TODO
    public void drive(float rightX, float rightY, float leftX, float leftY, boolean slowDrive) {
        // Prevent small values from causing the robot to drift
        if (Math.abs(rightX) < 0.01) {
            rightX = 0.0f;
        }
        if (Math.abs(leftX) < 0.01) {
            leftX = 0.0f;
        }
        if (Math.abs(leftY) < 0.01) {
            leftY = 0.0f;
        }
        if (Math.abs(rightY) < 0.01) {
            rightY = 0.0f;
        }

        // write the values to the motors 1
        double frontLeftPower = -leftY - rightX + leftX;
        double frontRightPower = -leftY + rightX - leftX;
        double backLeftPower = -leftY - rightX - leftX;
        double backRightPower = -leftY + rightX + leftX;

        motors.forEach((name, motor) -> {
            switch(name) {
                case FRONTRIGHT:
                    motor.setPower(Range.clip(frontRightPower, -1, 1));
                    break;
                case BACKLEFT:
                    motor.setPower(Range.clip(backLeftPower, -1, 1));
                    break;
                case FRONTLEFT:
                    motor.setPower(Range.clip(frontLeftPower, -1, 1));
                    break;
                case BACKRIGHT:
                    motor.setPower(Range.clip(backRightPower, -1, 1));
                    break;
            }
        });
    }

    private void driveToPositionTicks(int ticks, Direction direction, double maxPower) {
        if (!isStrafe(direction)) {
            int sign = (direction == Direction.FORWARD ? 1 : -1);
            for (DcMotor motor : motors.values()) {
                motor.setTargetPosition(motor.getCurrentPosition() + sign * ticks);
            }
        } else {
            int sign = (direction == Direction.RIGHT ? 1 : -1);
            motors.forEach((name, motor) -> {
                switch(name) {
                    case FRONTRIGHT:
                    case BACKLEFT:
                        motor.setTargetPosition(motor.getCurrentPosition() - sign * ticks);
                        break;
                    case FRONTLEFT:
                    case BACKRIGHT:
                        motor.setTargetPosition(motor.getCurrentPosition() + sign * ticks);
                        break;
                }
            });
        }

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(maxPower);
        double heading = -imuSystem.getHeading();

        setMotorPower(0.0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turn(heading + imuSystem.getHeading(), 0.5);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors.values()) {
            motor.setMode(runMode);
        }
    }

    /**
     * Gets the minimum distance from the target
     * @return
     */
    public int  getMinDistanceFromTarget() {
        int distance = Integer.MAX_VALUE;
        for (DcMotor motor : motors.values()) {
            distance = Math.min(distance, motor.getTargetPosition() - motor.getCurrentPosition());
        }
        return distance;
    }

    public void driveToPositionInches(double inches, Direction direction, double maxPower) {
        driveToPositionTicks(inchesToTicks(inches), direction, maxPower);
    }

    /**
     * Converts inches to ticks
     * @param inches Inches to convert to ticks
     * @return
     */
    public int inchesToTicks(double inches) {
        return (int) inches * TICKS_IN_INCH;
    }

    /**
     * Turns relative the heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turnAbsolute(double degrees, double maxPower) {
        turn(degrees, maxPower);
    }

    /**
     * Turns the robot by a given amount of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turn(double degrees, double maxPower) {
        double targetHeading = degrees + -imuSystem.getHeading();
        targetHeading = targetHeading % 360;

        if (targetHeading < 0) {
            targetHeading = targetHeading + 360;
        }

        double heading = -imuSystem.getHeading();
        double difference = computeDegreesDiff(targetHeading, heading);
        while (Math.abs(difference) > 1.0) {
            difference = computeDegreesDiff(targetHeading, heading);
            double power = getTurnPower(difference, maxPower);
            tankDrive(-power * Math.signum(difference), power * Math.signum(difference));
            heading = -imuSystem.getHeading();
        }
        this.setMotorPower(0);
    }

    /**
     * Causes the system to tank drive
     * @param leftPower sets the left side power of the robot
     * @param rightPower sets the right side power of the robot
     */
    public void tankDrive(double leftPower, double rightPower) {
        motors.forEach((name, motor) -> {
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setPower(leftPower);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setPower(rightPower);
                    break;
            }
        });
    }

    /**
     * Gets the turn power needed
     * @param degrees Number of degrees to turn
     * @return motor power from 0 - 0.8
     */
    private double getTurnPower(double degrees, double maxPower) {
        double power = Math.abs(degrees / 100.0);
        return Range.clip(power + 0.065, 0.0, maxPower);
    }

    private double computeDegreesDiff(double targetHeading, double heading) {
        double diff = targetHeading - heading;
        if (diff > 180) {
            return diff - 360;
        }
        if (diff < -180) {
            return 360 + diff;
        }
        return diff;
    }

    private boolean isStrafe(Direction direction) {
        return direction == Direction.LEFT || direction == Direction.RIGHT;
    }

}
