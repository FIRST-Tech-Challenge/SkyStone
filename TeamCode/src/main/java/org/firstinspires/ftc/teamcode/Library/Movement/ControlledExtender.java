/*
 * Drive exact based on the IMU and/or Encoders
 * partly based on org.firstinspires.ftc.robotcontroller.external.samples.PushbbotAutoDriveByEncoder_Linear
 */

package org.firstinspires.ftc.teamcode.Library.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class ControlledExtender {

    // Customized for the 435rpm (was 223) motor
    //static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    // TODO: Fix this...
    static final double     COUNTS_PER_MOTOR_REV    = 195.4 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    public static final double     COUNTS_PER_CM           = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;

    Telemetry telemetry;
    HardwareChassis robot;

    public ControlledExtender(HardwareChassis robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void start(double distance, double speed) {
        // Determine new target position
        double startPosition = robot.motor_extender.getCurrentPosition();
        double target = startPosition + COUNTS_PER_CM * distance;
        // And pass to motor controller
        robot.motor_extender.setTargetPosition((int) target);

        // Turn On RUN_TO_POSITION
        robot.motor_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        if (distance >= 0) {
            setMotors(speed);
        } else if (distance <= 0) {
            setMotors(-speed);
        }
    }

    public boolean endReached() {
        return !(robot.motor_extender.isBusy());
    }

    public void stop() {
        // Stop all motion;
        robot.motor_extender.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotors(double power) {
        robot.motor_extender.setPower(power);
    }
}