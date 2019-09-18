package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.IScale;
import org.firstinspires.ftc.teamcode.components.scale.LinearScale;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.base.System;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;
import org.firstinspires.ftc.teamcode.systems.logging.PhoneLogger;

public class DriveSystem {
    private final IScale JOYSTICK_SCALE = new LinearScale(0.62, 0);

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
    public PhoneLogger telemetry;

    private final int FORWARD = 1;
    private final int BACKWARD = 0;

    private double RAMP_POWER_CUTOFF = 0.3;
    private int RAMP_DISTANCE_TICKS;
    private final int TICKS_IN_INCH = 69;



    public DcMotor[] motors = new DcMotor[4];

    /**
     * Handles the data for the abstract creation of a drive system with four wheels
     * @param opMode opmode this system runs in
     */
    public DriveSystem(OpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = new PhoneLogger(opMode.telemetry);


        initMotors();

        imuSystem = new IMUSystem(opMode);
        initialImuHeading = imuSystem.getHeading();
        initImuPitch = imuSystem.getPitch();
        initImuRoll = imuSystem.getRoll();

        telemetry.log("DriveSystem","power: {0}", 0);
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

        // write the values to the motors 1
        double frontRightPower = -leftY + leftX - rightX;
        double backRightPower = -leftY - leftX - rightX;
        double frontLeftPower = -leftY - leftX + rightX;
        double backLeftPower = -leftY + leftX + rightX;


        this.motorFrontRight.setPower(Range.clip(frontRightPower, -1, 1));
        telemetry.log("Mecanum Drive System","FRpower: {0}", Range.clip(frontRightPower, -1, 1));
        this.motorBackRight.setPower(Range.clip(backRightPower, -1, 1));
        telemetry.log("Mecanum Drive System","BRPower: {0}", Range.clip(backRightPower, -1, 1));
        this.motorFrontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
        telemetry.log("Mecanum Drive System", "FLPower: {0}", Range.clip(frontLeftPower, -1, 1));
        this.motorBackLeft.setPower(Range.clip(backLeftPower, -1, 1));
        telemetry.log("Mecanum Drive System", "BLPower: {0}", Range.clip(backLeftPower, -1, 1));
        telemetry.write();
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





//    private void driveToPositionTicks(int ticks, double power, boolean shouldRamp) {
//        setMotorPower(0);
//
//        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
//        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);
//        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
//        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);
//
//        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        Ramp ramp = new ExponentialRamp(new Point(0, RAMP_POWER_CUTOFF),
//                new Point(RAMP_DISTANCE_TICKS, power));
//
//        double adjustedPower = Range.clip(power, -1.0, 1.0);
//        setMotorPower(adjustedPower);
//
//        while (anyMotorsBusy()) {
//            int distance = getMinDistanceFromTarget();
//
//            if (distance < 50) {
//                break;
//            }
//
//            double direction = 1.0;
//            if (distance < 0) {
//                distance = -distance;
//                direction = -1.0;
//            }
//
//            double scaledPower = shouldRamp ? ramp.scaleX(distance) : power;
//
//            setMotorPower(direction * scaledPower);
//            telemetry.log("MecanumDriveSystem", "distance left (ticks): " + getMinDistanceFromTarget());
//            telemetry.log("MecanumDriveSystem","scaled power: " + scaledPower);
//            telemetry.write();
//        }
//        setMotorPower(0);
//    }
//
//    public void setRunMode(DcMotor.RunMode runMode) {
//        for (DcMotor motor : motors) {
//            motor.setMode(runMode);
//        }
//    }
//
//    public void mecanumDriveXY(double x, double y) {
//        this.motorFrontRight.setPower(Range.clip(y + x, -1, 1));
//        this.motorBackRight.setPower(Range.clip(y - x, -1, 1));
//        this.motorFrontLeft.setPower(Range.clip(y - x, -1, 1));
//        this.motorBackLeft.setPower(Range.clip(y + x, -1, 1));
//    }
//
//    /**
//     * Checks if any of the motors are currently running
//     * @return Returns true if any motors are busy
//     */
//    public boolean anyMotorsBusy()
//    {
//        return motorFrontLeft.isBusy() ||
//                motorFrontRight.isBusy() ||
//                motorBackLeft.isBusy() ||
//                motorBackRight.isBusy();
//    }
//
//    /**
//     * Gets the minimum distance from the target
//     * @return
//     */
//    public int  getMinDistanceFromTarget() {
//        int d = this.motorFrontLeft.getTargetPosition() - this.motorFrontLeft.getCurrentPosition();
//        d = Math.min(d, this.motorFrontRight.getTargetPosition() - this.motorFrontRight.getCurrentPosition());
//        d = Math.min(d, this.motorBackLeft.getTargetPosition() - this.motorBackLeft.getCurrentPosition());
//        d = Math.min(d, this.motorBackRight.getTargetPosition() - this.motorBackRight.getCurrentPosition());
//        return d;
//    }
//
//    public void driveToPositionInches(int inches, double power, boolean shouldRamp) {
//         if (power <= 0 || inches <= 0) {
//             setMotorDirection(FORWARD);
//         } else {
//             setMotorDirection(BACKWARD);
//         }
//        int ticks = (int) inchesToTicks(inches);
//        driveToPositionTicks(ticks, power, shouldRamp);
//    }
//
//    /**
//     * Converts inches to ticks
//     * @param inches Inches to convert to ticks
//     * @return
//     */
//    public int inchesToTicks(int inches) {
//        return inches * TICKS_IN_INCH;
//    }

}
