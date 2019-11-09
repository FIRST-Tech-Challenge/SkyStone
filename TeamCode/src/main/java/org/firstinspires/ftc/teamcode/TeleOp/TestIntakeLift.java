package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

/*
 * -Gamepad A:
 *  -Left Joystick controls Left Wheel
 *  -Right Joystick controls Right Wheel
 *  -Left Bumper strafes left
 *  -Right Bumper strafes right
 *  -Y & B controls servo1
 *  -X & A controls servo2
 * -Gamepad B:
 *  -Y and A to intake and outake
 *  -Left joystick and Right joysticks to control the lift
 *  -Adjust the HardwareMap configurations in HardwareMap.java class
 *  -Adjust Intake/Lift power in teleopConstants.java class
 */

@TeleOp(name="Test Intake & Lift", group="Linear Opmode")
public class TestIntakeLift extends LinearOpMode {
    boolean intake = false;
    boolean outake = false;
    public void runOpMode(){
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            //------------------------------===Intake/Outake===------------------------------------------

            if(gamepad2.y && !intake) {
                intake = true;
                outake = false;
            } else if(gamepad2.y && intake){
                intake = false;
                outake = false;
            }

            if(gamepad2.a && !outake) {
                intake = false;
                outake = true;
            } else if(gamepad2.a && outake){
                intake = false;
                outake = false;
            }

            if(intake){
                hwMap.rightIntake.setPower(teleopConstants.intakePower);
                hwMap.leftIntake.setPower(-teleopConstants.intakePower);
            }

            if(outake){
                hwMap.rightIntake.setPower(-teleopConstants.intakePower);
                hwMap.leftIntake.setPower(teleopConstants.intakePower);
            }

            if(!intake && !outake){
                hwMap.rightIntake.setPower(0);
                hwMap.leftIntake.setPower(0);
            }

            //------------------------------===Linear Sliders===------------------------------------------

            if(gamepad2.left_stick_y == 1){
                hwMap.liftOne.setPower(teleopConstants.liftPower);
            } else if(gamepad2.left_stick_y == -1){
                hwMap.liftOne.setPower(-teleopConstants.liftPower);
            } else {
                hwMap.liftOne.setPower(0);
            }

            if(gamepad2.right_stick_y == 1){
                hwMap.liftTwo.setPower(-teleopConstants.liftPower);
            } else if(gamepad2.right_stick_y == -1){
                hwMap.liftTwo.setPower(teleopConstants.liftPower);
            } else {
                hwMap.liftTwo.setPower(0);
            }

            //------------------------------===Driving/Strafing===------------------------------------------

            if(gamepad1.left_stick_y == 1){
                hwMap.frontLeft.setPower(teleopConstants.drivePower);
                hwMap.backLeft.setPower(teleopConstants.drivePower);
            } else if(gamepad1.left_stick_y == -1){
                hwMap.frontLeft.setPower(-teleopConstants.drivePower);
                hwMap.backLeft.setPower(-teleopConstants.drivePower);
            } else {
                hwMap.frontLeft.setPower(0);
                hwMap.backLeft.setPower(0);
            }

            if(gamepad1.right_stick_y == 1){
                hwMap.frontRight.setPower(-teleopConstants.drivePower);
                hwMap.backRight.setPower(-teleopConstants.drivePower);
            } else if(gamepad1.right_stick_y == -1){
                hwMap.frontRight.setPower(teleopConstants.drivePower);
                hwMap.backRight.setPower(teleopConstants.drivePower);
            } else {
                hwMap.frontRight.setPower(0);
                hwMap.backRight.setPower(0);
            }

            if(gamepad1.left_bumper){
                hwMap.frontLeft.setPower(-teleopConstants.strafePower);     //FrontRight and BackLeft forwards, FrontLeft and BackRight backwards
                hwMap.backRight.setPower(teleopConstants.strafePower);
                hwMap.frontRight.setPower(-teleopConstants.strafePower);
                hwMap.backLeft.setPower(teleopConstants.strafePower);
            }

            if(gamepad1.right_bumper){
                hwMap.frontLeft.setPower(teleopConstants.strafePower);     //FrontLeft and BackRight forwards, FrontRight and BackLeft backwards
                hwMap.backRight.setPower(-teleopConstants.strafePower);
                hwMap.frontRight.setPower(teleopConstants.strafePower);
                hwMap.backLeft.setPower(-teleopConstants.strafePower);
            }
        }

        //------------------------------===Servos===------------------------------------------

        if(gamepad1.y)  //Change servo positions in teleopConstants.java
            hwMap.servo1.setPosition(teleopConstants.clawServo1Pos1);
        else if(gamepad1.b)
            hwMap.servo1.setPosition(teleopConstants.clawServo1Pos2);

        if(gamepad1.x)
            hwMap.servo2.setPosition(teleopConstants.clawServo2Pos1);
        else if(gamepad1.a)
            hwMap.servo2.setPosition(teleopConstants.clawServo2Pos2);
    }
}
