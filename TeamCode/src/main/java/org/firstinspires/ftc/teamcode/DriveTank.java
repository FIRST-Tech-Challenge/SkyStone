package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


// @Disabled
@TeleOp (name = "MMTeleOp")
public class DriveTank extends OpMode {

//    // drive motor declaration
//    DcMotor rearLeft;
//    DcMotor rearRight;
//    DcMotor frontLeft;
//    DcMotor frontRight;

    Robot robot = new Robot();

    @Override
    public void init() {

//        // Drive Motor instantiation
//        rearLeft = hardwareMap.dcMotor.get("rearLeft");
//        rearRight = hardwareMap.dcMotor.get("rearRight");
//
//        frontLeft = hardwareMap.dcMotor.get("frontLeft");
//        frontRight = hardwareMap.dcMotor.get("frontRight");
//
//        // Drive Motor Direction
//        rearLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        rearRight.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);

        robot.init(hardwareMap);

        telemetry.addData("Initialized", "Ready to start");

    }

    @Override
    public void loop() {
        double speedControl = 0.75; // to make the robot go slower
        // activate slowMode if both joysticks are pushed down
        boolean slowMode = gamepad1.left_stick_button && gamepad1.right_stick_button;

        if (slowMode) {
            speedControl = 0.5; // slowMode reduces the speed control
        }

        // Joystick
        double leftPower = speedControl * -gamepad1.left_stick_y;
        double rightPower = speedControl * -gamepad1.right_stick_y;
        double leftStrafe = speedControl * gamepad1.left_stick_x;
        double rightStrafe = speedControl * gamepad1.right_stick_x;

        // Strafe
        if (leftStrafe == rightStrafe && leftPower == 0 && rightPower == 0) {
            robot.rearLeft.setPower(-leftStrafe);
            robot.frontRight.setPower(-rightStrafe);

            robot.frontLeft.setPower(leftStrafe);
            robot.rearRight.setPower(rightStrafe);
        }

        // Not strafing
        else {
            robot.rearLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);

            robot.rearRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
        }
    }
}
