package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.TeleOp.teleopConstants;

@TeleOp(name = "TeleOpTest", group = "LinearOpMode")
public class testTeleop extends LinearOpMode {
    @Override
    public void runOpMode(){
        HardwareMap map = new HardwareMap(hardwareMap);
        boolean intake = false;
        int in = 0;
        int en = 0;

        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.left_trigger == 1){
                intake = true;
                en = 0;
                in = in + 1;
            } else if(gamepad2.right_trigger == 1){
                intake = false;
                in = 0;
                en = en + 1;
            }
            if(in == 2 || en == 2){
                in = 0;
                en = 0;
                map.intake.setPower(0);
            }
            if(intake && in > 0){
                map.intake.setPower(0.8);
            } else if(!intake && en > 0){
                map.intake.setPower(-0.8);
            }
            if(gamepad1.left_stick_y == 1){
                map.frontLeft.setPower(teleopConstants.power);
                map.backLeft.setPower(teleopConstants.power);
            } else if(gamepad1.left_stick_y == -1){
                map.frontLeft.setPower(-teleopConstants.power);
                map.backLeft.setPower(-teleopConstants.power);
            } else {
                map.frontLeft.setPower(0);
                map.backLeft.setPower(0);
            }
            if(gamepad1.right_stick_y == 1){
                map.frontRight.setPower(-teleopConstants.power);
                map.backRight.setPower(-teleopConstants.power);
            } else if(gamepad1.right_stick_y == -1){
                map.frontRight.setPower(teleopConstants.power);
                map.backRight.setPower(teleopConstants.power);
            } else {
                map.frontRight.setPower(0);
                map.backRight.setPower(0);
            }
            if(gamepad1.left_bumper){
                map.frontLeft.setPower(-2);
                map.backLeft.setPower(2);
                map.frontRight.setPower(-2);
                map.backRight.setPower(2);
            }
            if(gamepad1.right_bumper){
                map.frontLeft.setPower(2);
                map.backLeft.setPower(-2);
                map.frontRight.setPower(2);
                map.backRight.setPower(-2);
            }
            if(gamepad1.left_trigger == 1){
                teleopConstants.switchPower(0.6);
            } else if(gamepad1.right_trigger == 1) {
                teleopConstants.switchPower(1);
            }
            if(gamepad2.left_stick_y == 1){
                map.firstJoint.setPower(0.7);
            } else if(gamepad2.left_stick_y == -1){
                map.firstJoint.setPower(-0.7);
            } else{
                map.firstJoint.setPower(0);
            }
            if(gamepad2.right_stick_y == 1){
                map.secondJoint.setPower(-0.7);
            } else if(gamepad2.right_stick_y == -1){
                map.secondJoint.setPower(0.7);
            } else {
                map.secondJoint.setPower(0);
            }
            if(gamepad2.left_bumper){
                map.intakeJoint.setPower(0.5);
            } else {
                map.intakeJoint.setPower(0);
            }
            if(gamepad2.right_bumper){
                map.intakeJoint.setPower(-0.5);
            } else {
                map.intakeJoint.setPower(0);
            }
        }
    }
}
