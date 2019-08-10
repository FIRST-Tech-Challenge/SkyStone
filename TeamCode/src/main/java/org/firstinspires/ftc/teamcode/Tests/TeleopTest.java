package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.SixWheelDrivetrain;

@TeleOp
public class TeleopTest extends LinearOpMode {

    SixWheelDrivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new SixWheelDrivetrain(hardwareMap,this);
        waitForStart();

        while(opModeIsActive()){
            drivetrain.setLeftPower(gamepad1.left_stick_y);
            drivetrain.setRightPower(gamepad1.right_stick_y);
            drivetrain.updatePose();
            telemetry.addData("x: ", drivetrain.x);
            telemetry.addData("y: ", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();

        }


    }
}
