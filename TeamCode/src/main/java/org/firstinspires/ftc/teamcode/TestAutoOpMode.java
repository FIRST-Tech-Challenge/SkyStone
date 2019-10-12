// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test", group="Exercises")

public class TestAutoOpMode extends LinearOpMode {

    FourWheelsDriveBot robot = new FourWheelsDriveBot();
    static final double MOTOR_TICK_COUNT = 1120;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(),
                robot.leftRear.getCurrentPosition(),
                robot.rightRear.getCurrentPosition());
        telemetry.update();

        waitForStart();

        testOneMotor( robot.leftFront, 0.3, 1);
        testOneMotor( robot.rightFront, 0.3, 1);
        testOneMotor( robot.leftRear, 0.3, 1);
        testOneMotor( robot.rightRear,0.3, 1);

    }

    public void testOneMotor(DcMotor motor, double speed, int direction){
        // reset the timeout time and start motion.
        runtime.reset();

        double timeoutS = 5.0;
        // make 3 turn
        int target = motor.getCurrentPosition() + (int)MOTOR_TICK_COUNT * 3 * direction;
        telemetry.addData("Path2",  "Start %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition());
        telemetry.update();

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && motor.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running %s to %7d: @ %7d", motor.getDeviceName(), target, motor.getCurrentPosition());
            telemetry.update();
        }
        // Stop all motion;
        motor.setPower(0);

        telemetry.addData("Path2",  "Completed! %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition());
        telemetry.update();

        sleep(3000);

    }
}
