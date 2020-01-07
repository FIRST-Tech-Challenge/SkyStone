package org.firstinspires.ftc.teamcode.Library.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

public class ControlledDrive {
    static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CMS      = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    private Telemetry telemetry;
    HardwareChassis robot;

    public ControlledDrive(HardwareChassis robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void start(double distanceForward, double distanceSideways, double speed) {
        double maxDistance = Math.max(Math.abs(distanceForward), Math.abs(distanceSideways));

        double[] wheelSpeeds = OmniWheel.calculate(WHEEL_DIAMETER_CMS / 2, 38, 24, distanceForward / maxDistance, distanceSideways / maxDistance, 0);

        // Determine new target position
        double[] targets = {
                robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0] * (COUNTS_PER_CM * maxDistance),
                robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1] * (COUNTS_PER_CM * maxDistance),
                robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2] * (COUNTS_PER_CM * maxDistance),
                robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3] * (COUNTS_PER_CM * maxDistance)};
        // And pass to motor controller
        robot.motor_front_left.setTargetPosition((int) targets[0]);
        robot.motor_front_right.setTargetPosition((int) targets[1]);
        robot.motor_rear_left.setTargetPosition((int) targets[2]);
        robot.motor_rear_right.setTargetPosition((int) targets[3]);

        // Turn On RUN_TO_POSITION
        robot.motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.motor_front_left.setPower(wheelSpeeds[0] * speed);
        robot.motor_front_right.setPower(wheelSpeeds[1] * speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2] * speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3] * speed);
    }

    public boolean endReached() {
        return !(robot.motor_front_left.isBusy() || robot.motor_front_right.isBusy() || robot.motor_rear_left.isBusy() || robot.motor_rear_right.isBusy());
    }


    public void stop() {
        // Stop all motion;
        robot.motor_front_left.setPower(0);
        robot.motor_front_right.setPower(0);
        robot.motor_rear_left.setPower(0);
        robot.motor_rear_right.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
