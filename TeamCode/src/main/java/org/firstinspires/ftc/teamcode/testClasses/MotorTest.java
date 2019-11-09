package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() {

        telemetry.addLine("Init | v1.0");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("arm cur pos: ", arm.getCurrentPosition());
            //Other unit test code if you want
            telemetry.update();
        }
    }
}