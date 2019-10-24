package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

/*
 * -Y and A to intake and outake
 * -Left joystick and Right joysticks to control the lift
 * -Adjust the HardwareMap configurations in HardwareMap.java class
 * -Adjust Intake/Lift power in teleopConstants.java class
 */

@TeleOp(name="Test Intake & Lift", group="Linear Opmode")
public class TestIntakeLift extends LinearOpMode {
    boolean intake = false;
    boolean outake = false;
    public void runOpMode(){
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.y && !intake) {
                intake = true;
                outake = false;
            } else if(gamepad1.y && intake){
                intake = false;
                outake = false;
            }

            if(gamepad1.a && !outake) {
                intake = false;
                outake = true;
            } else if(gamepad1.a && outake){
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

            if(gamepad1.left_stick_y == 1){
                hwMap.liftOne.setPower(teleopConstants.liftPower);
            } else if(gamepad1.left_stick_y == -1){
                hwMap.liftOne.setPower(-teleopConstants.liftPower);
            } else {
                hwMap.liftOne.setPower(0);
            }

            if(gamepad1.right_stick_y == 1){
                hwMap.liftTwo.setPower(-teleopConstants.liftPower);
            } else if(gamepad1.right_stick_y == -1){
                hwMap.liftTwo.setPower(teleopConstants.liftPower);
            } else {
                hwMap.liftTwo.setPower(0);
            }
        }
    }
}
