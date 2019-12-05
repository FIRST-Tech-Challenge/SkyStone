/*
 * Drive exact based on the IMU and/or Encoders
 * partly based on org.firstinspires.ftc.robotcontroller.external.samples.PushbbotAutoDriveByEncoder_Linear
 */

package org.firstinspires.ftc.teamcode.Library.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

import java.util.function.Supplier;

public class ControlledArm {

    // Customized for the 435rpm motor
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 0.8;

    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;

    HardwareMap hardwareMap;
    HardwareChassis robot;

    public ControlledArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.robot = new HardwareChassis(hardwareMap);

        this.telemetry = telemetry;
    }

    public void raiseDistance(double distance, double speed, int timeout) {
        /*
         // Determine new target position
        double startPositionLeft = robot.motor_clamp_extender_left.getCurrentPosition();
        double startPositionRight = robot.motor_clamp_extender_left.getCurrentPosition();
        double targetLeft = startPositionLeft + COUNTS_PER_CM*distance;
        double targetRight = startPositionRight + COUNTS_PER_CM*distance;
        // And pass to motor controller
        robot.motor_clamp_extender_left.setTargetPosition((int) targetLeft);
        robot.motor_clamp_extender_right.setTargetPosition((int) targetRight);

        // Turn On RUN_TO_POSITION
        robot.motor_clamp_extender_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_clamp_extender_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        this.runtime.reset();
        if (distance >= 0) {
            robot.motor_clamp_extender_left.setPower(speed);
            robot.motor_clamp_extender_right.setPower(speed);
        } else if (distance <= 0) {
            robot.motor_clamp_extender_left.setPower(-speed);
            robot.motor_clamp_extender_right.setPower(-speed);
        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeout) && robot.motor_clamp_extender_left.isBusy() && robot.motor_clamp_extender_right.isBusy()) {
            double countsLeft = robot.motor_clamp_extender_left.getCurrentPosition() - targetLeft;
            this.telemetry.addData("Position", countsLeft/COUNTS_PER_CM);
            this.telemetry.update();
        }

        // Stop all motion;
        robot.motor_clamp_extender_left.setPower(0);
        robot.motor_clamp_extender_right.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor_clamp_extender_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_clamp_extender_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

         
    }

    // driveCondition(1,1, () -> button.getState() == false);
    public void raiseConditionally(double speedForward, double speedSideways, Supplier<Boolean> condition) {
        while (condition.get()) {
            //drive
        }
    }
}