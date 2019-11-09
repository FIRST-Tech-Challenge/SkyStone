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
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("arm pos: ", arm.getCurrentPosition());
            if(gamepad1.a){
                arm.setTargetPosition(arm.getTargetPosition()+1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(gamepad1.a);
            }
            if(gamepad1.b){
                arm.setTargetPosition(arm.getTargetPosition()-1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(gamepad1.b);
            }
            telemetry.addData("arm set pos: ", arm.getTargetPosition());
            telemetry.addData("arm cur pos: ", arm.getCurrentPosition());
            //Other unit test code if you want
            telemetry.update();
        }
    }
}