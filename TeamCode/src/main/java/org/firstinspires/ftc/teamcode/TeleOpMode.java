package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "TeleOpMode", group = "Teleop")
public class TeleOpMode extends LinearOpMode {
    public void runOpMode() {
        DcMotor hook = hardwareMap.dcMotor.get("hook");
        DcMotor frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        DcMotor backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        telemetry.addData("Init","v:1.0");
        waitForStart();
        
        while (opModeIsActive()) {
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
            //Chassis test
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            frontLeftDrive.setPower(v1);
            frontRightDrive.setPower(v2);
            backLeftDrive.setPower(v3);
            backRightDrive.setPower(v4);
            telemetry.addData("robot angle: ", robotAngle);
            telemetry.addData("hook position: ", hook.getCurrentPosition());
            telemetry.update();
        }
    }
}  
