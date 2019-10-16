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
            if (gamepad1.y) {
                rob.servo.setPosition(1);
            } else if (gamepad1.a) {
                rob.servo.setPosition(0);
            }

            if(gamepad1.right_stick_y > 0.1){
                rob.driveTrainMovement(0.8, Crane.movements.forward);
            }else if(gamepad1.right_stick_y > -0.1){
                rob.driveTrainMovement(0.8, Crane.movements.backward);
            }else{
                rob.stopDrivetrain();
            }

            if(gamepad1.right_stick_x > 0.1){
                rob.driveTrainMovement(0.8, Crane.movements.right);
            }else if(gamepad1.right_stick_x > -0.1){
                rob.driveTrainMovement(0.8, Crane.movements.left);
            }else{
                rob.stopDrivetrain();
            }

            if(gamepad1.dpad_left){
                rob.rack.setPower(-0.1);
            }else if(gamepad1.dpad_right){
                rob.rack.setPower(0.1);
            }else{
                rob.rack.setPower(0);
            }

            if(gamepad1.dpad_up){
                rob.linear.setPower(0.1);
            }else if(gamepad1.dpad_down){
                rob.linear.setPower(0.1);
            }else{
                rob.linear.setPower(0);
            }
        }
    }
}
