// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Encoder Final", group="Exercises")

public class FourWDEncoder extends LinearOpMode {

    HardwareBot robot = new HardwareBot();
    static final double MOTOR_TICK_COUNT = 1120;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        waitForStart();

        robot.leftFront.setPower(1);
        robot.rightFront.setPower(-1);
        robot.leftRear.setPower(1);
        robot.rightRear.setPower(1);
        sleep(500);

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

        double quarterTurn = 10/4;

        robot.heavyDutyArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget = robot.heavyDutyArm.getTargetPosition() + (int)quarterTurn;
        robot.heavyDutyArm.setTargetPosition(newTarget);
        robot.heavyDutyArm.setPower(0.5);
        robot.heavyDutyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.heavyDutyArm.isBusy()) {
            telemetry.addData("Status", "Running the motor to a quarter turn.");
            telemetry.update();
        }

        robot.heavyDutyArm.setPower(0);
        robot.heavyDutyArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTarget = robot.heavyDutyArm.getTargetPosition() - (int)quarterTurn;
        robot.heavyDutyArm.setTargetPosition(newTarget);
        robot.heavyDutyArm.setPower(0.5);
        robot.heavyDutyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.heavyDutyArm.isBusy()) {
            telemetry.addData("Status", "Running the motor BACK a quarter turn.");
            telemetry.update();
        }

        robot.heavyDutyArm.setPower(0);

        newTarget = robot.heavyDutyArm.getTargetPosition() + (int)MOTOR_TICK_COUNT*3;
        robot.heavyDutyArm.setTargetPosition(newTarget);
        robot.heavyDutyArm.setPower(1);
        robot.heavyDutyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.heavyDutyArm.isBusy()) {
            telemetry.addData("Status", "Running the motor BACK a quarter turn.");
            telemetry.update();
        }

        sleep(5000);
        robot.heavyDutyArm.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
