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
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareOmniTest;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

import java.util.function.Supplier;

public class ControlledDrive {
    static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CMS      = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry;

    HardwareMap hardwareMap;
    HardwareChassis robot;

    public ControlledDrive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.robot = new HardwareChassis(hardwareMap);
    }

    public void driveDistance(double distanceForward, double distanceSideways, double speed, int timeout) {
        double maxDistance = Math.max(Math.abs(distanceForward), Math.abs(distanceSideways));

        double[] wheelSpeeds =  OmniWheel.calculate(WHEEL_DIAMETER_CMS/2, 38, 24, distanceForward/maxDistance ,distanceSideways/maxDistance, 0);

        // Determine new target position
        double[] targets = {
                robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0]*(COUNTS_PER_CM*maxDistance),
                robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1]*(COUNTS_PER_CM*maxDistance),
                robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2]*(COUNTS_PER_CM*maxDistance),
                robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3]*(COUNTS_PER_CM*maxDistance)};
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
        runtime.reset();
        robot.motor_front_left.setPower(wheelSpeeds[0] * speed);
        robot.motor_front_right.setPower(wheelSpeeds[1] * speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2] * speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3] * speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeout) &&
                (robot.motor_front_left.isBusy() || robot.motor_front_right.isBusy() || robot.motor_rear_left.isBusy() || robot.motor_rear_right.isBusy())) {
            //sleep(1);
        }

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

    // driveCondition(1,1, () -> button.getState() == false);

    /**
     * drive in any direction based on given parameters ( 1 to -1)
     * @param speedForward
     * @param speedSideways
     * @param condition
     */
    public void driveConditionally(double speedForward, double speedSideways, Supplier<Boolean> condition) {
        while (condition.get()) {
            //drive
            double[] result = OmniWheel.calculate(5.0, 38, 24, -speedForward, speedSideways, 0);

            robot.motor_front_left.setPower(result[0]);
            robot.motor_front_right.setPower(result[1]);
            robot.motor_rear_left.setPower(result[2]);
            robot.motor_rear_right.setPower(result[3]);
        }
    }

    public void rotate(double degree, double speed, int timeout) {

    }
}
