package org.firstinspires.ftc.teamcode.testClasses;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import java.io.IOException;
import java.util.HashMap;

@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        Chassis chassis;
        try {
            chassis = new Chassis(hardwareMap, "/res/config/chassisMotors.txt");
        } catch (IOException e) {
            chassis = new Chassis();
            e.printStackTrace();
        }
        //DcMotor hook = hardwareMap.dcMotor.get("hook");
        //DcMotor leftIntake = hardwareMap.dcMotor.get("left_intake");
        //DcMotor rightIntake = hardwareMap.dcMotor.get("right_intake");
        telemetry.addData("Init","v:1.0");
        waitForStart();
        
        while (opModeIsActive()) {
            /*
            //Hook test
            if (gamepad1.a == true) {
                hook.setPower(1);
            }
            else if (gamepad1.b == true) {
                hook.setPower(-1);
            }
            else {
                hook.setPower(0);
            }
            //Intake test
            if (gamepad1.x == true) {
                leftIntake.setPower(1);
                rightIntake.setPower(-1);
            }
            else if (gamepad1.y == true) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            }
            else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
             */
            //Chassis test
            double stickRadius = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double targetAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double turnPower = gamepad1.right_stick_x;
            final double frontLeftPower = stickRadius * Math.cos(targetAngle) + turnPower;
            final double frontRightPower = stickRadius * Math.sin(targetAngle) - turnPower;
            final double backLeftPower = stickRadius * Math.sin(targetAngle) + turnPower;
            final double backRightPower = stickRadius * Math.cos(targetAngle) - turnPower;
            HashMap<String, Double> chassisPowers = new HashMap<String, Double>();
            chassisPowers.put("frontLeft", frontRightPower);
            chassisPowers.put("frontRight", frontRightPower);
            chassisPowers.put("backLeft", backLeftPower);
            chassisPowers.put("backRight", backRightPower);
            chassis.setMotors(chassisPowers);

            //Telemetry
            telemetry.update();
        }
    }
}  
