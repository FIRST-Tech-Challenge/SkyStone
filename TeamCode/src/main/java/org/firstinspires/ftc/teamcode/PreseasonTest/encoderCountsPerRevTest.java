package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

@TeleOp(name="EncoderTester", group="Linear Opmode")
public class encoderCountsPerRevTest extends LinearOpMode {
    boolean onA = false;
    boolean onB = false;
    @Override
    public void runOpMode() throws InterruptedException{
        HardwareMap map = new HardwareMap(hardwareMap);
        telemetry.addData("Instructions:","A to start moving/stop");
        telemetry.addData("Instructions:","B to Display telemetry");
        telemetry.addData("Instructions:","X to Reset Encoder counts");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                map.frontLeft.setPower(0.3);
                map.frontRight.setPower(-0.3);
                map.backLeft.setPower(0.3);
                map.backRight.setPower(-0.3);
            } else {
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
            }
            if(gamepad1.b && !onB){
                telemetry.clear();
                telemetry.addData(map.TAG,"Data will now be shown");
                telemetry.update();
                onB = true;
            } else if(gamepad1.b && onB){
                telemetry.addData(map.TAG,"Data turned off");
                telemetry.update();
                onB = false;
            }
            if(gamepad1.x){
                map.resetEncoders();
                telemetry.clear();
                telemetry.addData(map.TAG, "frontRight: " + map.frontRight.getCurrentPosition());
                telemetry.addData(map.TAG, "frontLeft: " + map.frontLeft.getCurrentPosition());
                telemetry.addData(map.TAG, "backRight: " + map.backRight.getCurrentPosition());
                telemetry.addData(map.TAG, "backLeft: " + map.backLeft.getCurrentPosition());
                telemetry.update();
            }
            if(onB){
                telemetry.clear();
                telemetry.addData(map.TAG, "frontRight: " + map.frontRight.getCurrentPosition());
                telemetry.addData(map.TAG, "frontLeft: " + map.frontLeft.getCurrentPosition());
                telemetry.addData(map.TAG, "backRight: " + map.backRight.getCurrentPosition());
                telemetry.addData(map.TAG, "backLeft: " + map.backLeft.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
