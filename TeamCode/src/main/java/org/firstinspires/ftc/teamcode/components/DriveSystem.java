package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import java.util.EnumMap;

public class DriveSystem {

    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;

        private static boolean isStrafe(Direction direction) {
            return direction == LEFT || direction == RIGHT;
        }
    }

    public int counter = 0;

    public static final String TAG = "DriveSystem";

    public EnumMap<MotorNames, DcMotor> motors;

    public IMUSystem imuSystem;

    private int mTargetTicks;
    private double mTargetHeading;

    private final double TICKS_IN_MM = 2.716535433;

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     */
    public DriveSystem(EnumMap<MotorNames, DcMotor> motors, BNO055IMU imu) {
        this.motors = motors;
        mTargetTicks = 0;
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    break;
            }
        });

        setMotorPower(0);
    }

    /**
     * Clips joystick values and drives the motors.
     * @param rightX Right X joystick value
     * @param leftX Left X joystick value
     * @param leftY Left Y joystick value in case you couldn't tell from the others
     */
    // TODO
    public void drive(float rightX, float leftX, float leftY) {
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

        double frontLeftPower = -leftY + rightX + leftX;
        double frontRightPower = -leftY - rightX - leftX;
        double backLeftPower = -leftY + rightX - leftX;
        double backRightPower = -leftY - rightX + leftX;

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

    public boolean driveToPositionTicks(int ticks, Direction direction, double maxPower) {
        if(mTargetTicks == 0){
            mTargetTicks = direction == Direction.BACKWARD ? -ticks : ticks;
            motors.forEach((name, motor) -> {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                if(Direction.isStrafe(direction)){
                    int sign = direction == Direction.LEFT ? -1 : 1;
                    switch(name){
                        case FRONTLEFT:
                        case BACKRIGHT:
                            motor.setTargetPosition(sign * mTargetTicks);
                            break;
                        case FRONTRIGHT:
                        case BACKLEFT:
                            motor.setTargetPosition(sign * -mTargetTicks);
                            break;
                    }
                } else {
                    motor.setTargetPosition(mTargetTicks);
                }
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(maxPower);
            });
        }

        for (DcMotor motor : motors.values()) {
            int offset = Math.abs(motor.getCurrentPosition() - mTargetTicks);
            if(offset <= 0){

                // Shut down motors
                setMotorPower(0);

                // Reset motors to default run mode
                setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Reset target
                mTargetTicks = 0;

                // Motor has reached target
                return true;
            }
        }

        // Motor has not reached target
        return false;

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

    public boolean driveToPosition(int millimeters, Direction direction, double maxPower) {
        return driveToPositionTicks(millimetersToTicks(millimeters), direction, maxPower);
    }

    /**
     * Converts millimeters to ticks
     * @param millimeters Millimeters to convert to ticks
     * @return
     */
    public int millimetersToTicks(int millimeters) {
        return (int) Math.round(millimeters * TICKS_IN_MM);
    }

    /**
     * Turns relative the heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turnAbsolute(double degrees, double maxPower) {
        turn(imuSystem.getHeading() + degrees, maxPower);
    }

    /**
     * Turns the robot by a given number of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public boolean turn(double degrees, double maxPower) {

        double heading = imuSystem.getHeading();
        Log.d(TAG,"Current Heading: " + heading);
        if(mTargetHeading == 0){

              mTargetHeading = (heading + degrees) % 360;
            Log.d(TAG,"Setting Heading -- Target: " + mTargetHeading);

            Log.d(TAG, "Degrees: " + degrees);
//            if (targetHeading < 0) {
//                targetHeading = targetHeading + 360;
//            }
//            mTargetHeading = targetHeading;
        }



//        double difference = computeDegreesDiff(mTargetHeading, heading);
        double difference = mTargetHeading - heading;
        Log.d(TAG,"Difference: " + difference);

        if (Math.abs(difference) < 1.0) {
            setMotorPower(0);
            mTargetHeading = 0;
            return true;
        }

        double power = getTurnPower(difference, maxPower);
        Log.d(TAG,"Turn Power: " + power);
        tankDrive(power, -power);

        return false;

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
       // double power = Math.abs(degrees / 100.0);
        return Range.clip(degrees / 100.0, -maxPower, maxPower);
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
}
