package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Basic Auton 1", group = "basic")
public class autoblue1 extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            if(n==1){
                rob.rightLinear.setPower(0.6);
                rob.leftLinear.setPower(0.6);
                wait(1000);
                rob.rightLinear.setPower(0);
                rob.leftLinear.setPower(0);

                rob.driveTrainMovement(2, Crane.movements.forward);
                wait(3000);
                rob.stopDrivetrain();

                while(!rob.flimit.getState()) {
                    rob.rack.setPower(0.2);
                }
                rob.rack.setPower(0);

                rob.rightLinear.setPower(-0.6);
                rob.leftLinear.setPower(-0.6);
                wait(1000);
                rob.rightLinear.setPower(0);
                rob.leftLinear.setPower(0);

                rob.rightServo.setPosition(rob.rightServo.getPosition() + 2);
                rob.leftServo.setPosition(rob.leftServo.getPosition() + 2);

                rob.rightServo.setPosition(rob.rightServo.getPosition() - 0.01);
                rob.leftServo.setPosition(rob.leftServo.getPosition() - 0.01);


            }
            else if(n==0){

            }
            else{

            }
        }


    }
}
