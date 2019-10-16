package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Crane;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "Rack Test", group = "Concept")



public class RackTest extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive, Crane.setupType.claw);

        while (opModeIsActive()){

            if(gamepad1.x){
                rob.rack.setPower(-0.25);
            }else if(gamepad1.b){
                rob.rack.setPower(0.25);
            }else{
                rob.rack.setPower(0);
            }

            if(gamepad1.y){
                rob.linear.setPower(-0.5);
            }else if(gamepad1.a){
                rob.linear.setPower(0.5);
            }else{
                rob.linear.setPower(0);
            }
/*
            if(gamepad1.dpad_up){
                rob.driveTrainMovement(0.5, Crane.movements.forward);
            }else if(gamepad1.dpad_down){
                rob.driveTrainMovement(0.5, Crane.movements.backward);
            }else if(gamepad1.dpad_right){
                rob.driveTrainMovement(0.5, Crane.movements.right);
            }else if(gamepad1.dpad_left){
                rob.driveTrainMovement(0.5, Crane.movements.left);
            }else{
                rob.stopDrivetrain();
            }

 */
        }
    }
}
