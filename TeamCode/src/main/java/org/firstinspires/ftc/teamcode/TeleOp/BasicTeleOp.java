package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "TeleOp", group = "Concept")



public class BasicTeleOp extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive, Crane.setupType.claw);

        while (opModeIsActive()){
            standardGamepadData();

/*
            if (gamepad1.y) {
                rob.rightServo.setPosition(1);
                rob.leftServo.setPosition(1);
            } else if (gamepad1.a) {
                rob.rightServo.setPosition(0);
                rob.leftServo.setPosition(0);
            }

            if(gamepad1.b){
                rob.rotationservo.setPosition(0);
            }else if (gamepad1.x) {
                rob.rotationservo.setPosition(1);
            }

 */

            if(g(0)){
                rob.driveTrainMovement(0.8, Crane.movements.forward);
            }else if(g(2)){
                rob.driveTrainMovement(0.8, Crane.movements.backward);
            }else if(g(3)){
                rob.driveTrainMovement(0.8, Crane.movements.right);
            }else if(g(1)) {
                rob.driveTrainMovement(0.8, Crane.movements.left);
            }else if(g(8)){
                    rob.driveTrainMovement(0.8, Crane.movements.ccw);
            }else if(g(9)){
                rob.driveTrainMovement(0.8, Crane.movements.cw);
            }else{
                rob.stopDrivetrain();
            }

            if(gamepad1.dpad_left){
                if(!rob.flimit.getState()) {
                    rob.rack.setPower(0.2);
                }
            }else if(gamepad1.dpad_right) {
                if (!rob.blimit.getState()){
                    rob.rack.setPower(-0.2);
                }
            }else{
                rob.rack.setPower(0);
            }

            if(gamepad1.dpad_up){
                rob.linear.setPower(0.6);
            }else if(gamepad1.dpad_down){
                rob.linear.setPower(-0.6);
            }else {
                rob.linear.setPower(0);
            }

        }
    }
}
