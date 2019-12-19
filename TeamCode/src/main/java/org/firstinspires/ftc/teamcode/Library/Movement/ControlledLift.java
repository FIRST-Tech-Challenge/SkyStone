/*
 * Drive exact based on the IMU and/or Encoders
 * partly based on org.firstinspires.ftc.robotcontroller.external.samples.PushbbotAutoDriveByEncoder_Linear
 */

package org.firstinspires.ftc.teamcode.Library.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

import java.util.function.Supplier;

public class ControlledLift {
    // Customized for the 435rpm motor
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 0.8;

    private Telemetry telemetry;

    HardwareChassis robot;

    public ControlledLift(HardwareChassis robot, Telemetry telemetry) { //
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void start(double distance, double speed) {
        // Determine new target position
        double startPositionLeft = robot.motor_lift_left.getCurrentPosition();
        double startPositionRight = robot.motor_lift_left.getCurrentPosition();
        double targetLeft = startPositionLeft + COUNTS_PER_CM * distance;
        double targetRight = startPositionRight + COUNTS_PER_CM * -distance;
        // And pass to motor controller
        robot.motor_lift_left.setTargetPosition((int) targetLeft);
        robot.motor_lift_right.setTargetPosition((int) targetRight);

        // Turn On RUN_TO_POSITION
        robot.motor_lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        if (distance >= 0) {
            setMotors(speed);
        } else if (distance <= 0) {
            setMotors(-speed);
        }
    }

    public boolean endReached() {
        return !(robot.motor_lift_left.isBusy() || robot.motor_lift_right.isBusy());
    }

    public void stop() {
        // Stop all motion;
        robot.motor_lift_left.setPower(0);
        robot.motor_lift_right.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor_lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotors(double power) {
        robot.motor_lift_left.setPower(power);
        robot.motor_lift_right.setPower(-power);
    }
}