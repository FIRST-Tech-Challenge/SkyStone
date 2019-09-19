package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.LinearScale;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSystem {
    private final LinearScale JOYSTICK_SCALE = new LinearScale(0.62, 0);

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private boolean slowDrive;
    public IMUSystem imuSystem;
    private double initialImuHeading;
    private double initImuPitch;
    private double initImuRoll;

    protected HardwareMap hardwareMap;

    private final int FORWARD = 1;
    private final int BACKWARD = 0;
    private final int STRAFE_LEFT = 0;
    private final int STRAFE_RIGHT = 1;

    private double RAMP_POWER_CUTOFF = 0.3;
    private int RAMP_DISTANCE_TICKS;
    private final int TICKS_IN_INCH = 69;
    private double TURN_RAMP_POWER_CUTOFF = 0.1;

    private Telemetry telemetry;



    public DcMotor[] motors = new DcMotor[4];

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     * @param opMode opmode this system runs in
     */
    public DriveSystem(OpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;


        initMotors();

        imuSystem = new IMUSystem(opMode);
        initialImuHeading = imuSystem.getHeading();
        initImuPitch = imuSystem.getPitch();
        initImuRoll = imuSystem.getRoll();
    }

    /**
     * Set the power of the drive system
     * @param power power of the system
     */
    public void setMotorPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    /**
     * Sets the direction of all the motors
     * @param direction The new direction of the motors
     */
    public void setMotorDirection(int direction) {
        switch (direction){
            case FORWARD:
                motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case BACKWARD:
                motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
        }
    }

    public void initMotors() {

        this.motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        this.motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        this.motorBackRight = hardwareMap.dcMotor.get("motorBR");
        this.motorBackLeft = hardwareMap.dcMotor.get("motorBL");

        motors[0] = motorFrontLeft;
        motors[1] = motorFrontRight;
        motors[2] = motorBackRight;
        motors[3] = motorBackLeft;

        setMotorDirection(FORWARD);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

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
    public void drive(float rightX, float rightY, float leftX, float leftY, boolean slowDrive) {
        this.slowDrive = slowDrive;
        setMotorDirection(FORWARD);


        rightX = Range.clip(rightX, -1, 1);
        leftX = Range.clip(leftX, -1, 1);
        leftY = Range.clip(leftY, -1, 1);

        rightX = scaleJoystickValue(rightX);
        leftX = scaleJoystickValue(leftX);
        leftY = scaleJoystickValue(leftY);

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
        double frontRightPower = -leftY + leftX - rightX;
        double backRightPower = -leftY + leftX + rightX;
        double frontLeftPower = -leftY - leftX + rightX;
        double backLeftPower = -leftY - leftX - rightX;


        this.motorFrontRight.setPower(Range.clip(frontRightPower, -1, 1));
        telemetry.addData("Mecanum Drive System","FRpower: {0}", Range.clip(frontRightPower, -1, 1));
        this.motorBackRight.setPower(Range.clip(backRightPower, -1, 1));
        telemetry.addData("Mecanum Drive System","BRPower: {0}", Range.clip(backRightPower, -1, 1));
        this.motorFrontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
        telemetry.addData("Mecanum Drive System", "FLPower: {0}", Range.clip(frontLeftPower, -1, 1));
        this.motorBackLeft.setPower(Range.clip(backLeftPower, -1, 1));
        telemetry.addData("Mecanum Drive System", "BLPower: {0}", Range.clip(backLeftPower, -1, 1));
        telemetry.update();
    }

    /**
     * Scales the joystick value while keeping in mind slow mode.
     * @param joystickValue
     * @return a value from 0 - 1 based on the given value
     */
    private float scaleJoystickValue(float joystickValue) {
        float slowDriveCoefficient = .3f;
        if (!slowDrive) slowDriveCoefficient = 1;
        return joystickValue > 0
                ? (float)  JOYSTICK_SCALE.scaleX(joystickValue * joystickValue) * slowDriveCoefficient
                : (float) -JOYSTICK_SCALE.scaleX(joystickValue * joystickValue) * slowDriveCoefficient;
    }





    private void driveToPositionTicks(int ticks, double power, boolean shouldRamp) {
        setMotorPower(0);

        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp ramp = new ExponentialRamp(new Point(0, RAMP_POWER_CUTOFF),
                new Point(RAMP_DISTANCE_TICKS, power));

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        setMotorPower(adjustedPower);

        while (anyMotorsBusy()) {
            int distance = getMinDistanceFromTarget();

            if (distance < 50) {
                break;
            }

            double direction = 1.0;
            if (distance < 0) {
                distance = -distance;
                direction = -1.0;
            }

            double scaledPower = shouldRamp ? ramp.scaleX(distance) : power;

            setMotorPower(direction * scaledPower);
            telemetry.addData("MecanumDriveSystem", "distance left (ticks): " + getMinDistanceFromTarget());
            telemetry.addData("MecanumDriveSystem","scaled power: " + scaledPower);
            telemetry.update();
        }
        setMotorPower(0);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void mecanumDriveXY(double x, double y) {
        this.motorFrontRight.setPower(Range.clip(y + x, -1, 1));
        this.motorBackRight.setPower(Range.clip(y - x, -1, 1));
        this.motorFrontLeft.setPower(Range.clip(y - x, -1, 1));
        this.motorBackLeft.setPower(Range.clip(y + x, -1, 1));
    }

    /**
     * Checks if any of the motors are currently running
     * @return Returns true if any motors are busy
     */
    public boolean anyMotorsBusy()
    {
        return motorFrontLeft.isBusy() ||
                motorFrontRight.isBusy() ||
                motorBackLeft.isBusy() ||
                motorBackRight.isBusy();
    }

    /**
     * Gets the minimum distance from the target
     * @return
     */
    public int  getMinDistanceFromTarget() {
        int d = this.motorFrontLeft.getTargetPosition() - this.motorFrontLeft.getCurrentPosition();
        d = Math.min(d, this.motorFrontRight.getTargetPosition() - this.motorFrontRight.getCurrentPosition());
        d = Math.min(d, this.motorBackLeft.getTargetPosition() - this.motorBackLeft.getCurrentPosition());
        d = Math.min(d, this.motorBackRight.getTargetPosition() - this.motorBackRight.getCurrentPosition());
        return d;
    }

    public void driveToPositionInches(int inches, double power, boolean shouldRamp) {
         if (power <= 0 || inches <= 0) {
             setMotorDirection(FORWARD);
         } else {
             setMotorDirection(BACKWARD);
         }
        int ticks = (int) inchesToTicks(inches);
        driveToPositionTicks(ticks, power, shouldRamp);
    }

    /**
     * Converts inches to ticks
     * @param inches Inches to convert to ticks
     * @return
     */
    public int inchesToTicks(int inches) {
        return inches * TICKS_IN_INCH;
    }

    /**
     * Strafes left for the given amount of inches
     * @param inches Amount of inches to strafe
     * @param power Power of the motors
     */
    public void strafeLeftToPositionInches(int inches, double power) {
        setMotorDirection(STRAFE_LEFT);
        int ticks = inchesToTicks(inches);
        driveToPositionTicks(ticks, power, true);
    }
    /**
     * Strafes right for the given amount of inches
     * @param inches Amount of inches to strafe
     * @param power Power of the motors
     */
    public void strafeRightToPositionInches(int inches, double power) {
        setMotorDirection(STRAFE_RIGHT);
        int ticks =  inchesToTicks(inches);
        driveToPositionTicks(ticks, power, true);
    }



    /**
     * Turns relative the heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turnAbsolute(double degrees, double maxPower) {
        turn(degrees, maxPower, initialImuHeading);
    }

    /**
     * Turns the robot by a given amount of degrees using the current heading
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turn(double degrees, double maxPower) {
        turn(degrees, maxPower, imuSystem.getHeading());
    }

    /**
     * Turns the robot by a given amount of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     * @param initialHeading The initial starting point
     */
    private void turn(double degrees, double maxPower, double initialHeading) {
        setMotorDirection(FORWARD);

        double heading = -initialHeading;
        double targetHeading = 0;

        if ((degrees % 360) > 180) {
            targetHeading = heading + ((degrees % 360) - 360);
        } else {
            targetHeading = heading + (degrees % 360);
        }

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Between 90 (changed from 130) and 2 degrees away from the target
        // we want to slow down from maxPower to 0.1
        ExponentialRamp ramp = new ExponentialRamp(new Point(2.0, TURN_RAMP_POWER_CUTOFF), new Point(90, maxPower));

        while (Math.abs(computeDegreesDiff(targetHeading, heading)) > 1) {
            double power = getTurnPower(ramp, targetHeading, heading);
            telemetry.addData("MecanumDriveSystem","heading: " + heading);
            telemetry.addData("MecanumDriveSystem","target heading: " + targetHeading);
            telemetry.addData("MecanumDriveSystem","power: " + power);
            telemetry.addData("MecanumDriveSystem","distance left: " + Math.abs(targetHeading - heading));
            telemetry.update();


            tankDrive(power, -power);
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
        this.motorFrontLeft.setPower(leftPower);
        this.motorBackLeft.setPower(leftPower);
        this.motorFrontRight.setPower(rightPower);
        this.motorBackRight.setPower(rightPower);
    }

    /**
     * Gets the turn power needed
     * @param ramp the ramp
     * @param targetHeading the target heading
     * @param heading the heading
     * @return
     */

    private double getTurnPower(Ramp ramp, double targetHeading, double heading) {
        double diff = computeDegreesDiff(targetHeading, heading);

        if (diff < 0) {
            return -ramp.scaleX(Math.abs(diff));
        } else {
            return ramp.scaleX(Math.abs(diff));
        }
    }

    private double computeDegreesDiff(double targetHeading, double heading) {
        return targetHeading - heading;
    }

}
