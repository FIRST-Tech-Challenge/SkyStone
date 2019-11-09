package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Basic Auton", group = "basic")
public class basicblue extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            rob.rightLinear.setPower(0.6);
            rob.leftLinear.setPower(0.6);
            wait(1000);
            rob.rightLinear.setPower(0);
            rob.leftLinear.setPower(0);
            /*
            if(!rob.flimit.getState()) {
                rob.rack.setPower(0.2);
            }
            rob.rack.setPower(0);

            rob.driveTrainMovement(10, Crane.movements.forward);
            wait(5000);
            rob.stopDrivetrain();

            rob.driveTrainMovement(10, Crane.movements.right);
            wait(2000);
            rob.stopDrivetrain();

            rob.rotationservo.setPosition(90);

            rob.driveTrainMovement(10, Crane.movements.left);
            wait(2000);
            rob.stopDrivetrain();

            rob.rotationservo.setPosition(0);

            rob.driveTrainMovement(10, Crane.movements.backward);
            wait(2500);
            rob.stopDrivetrain();
            */
        }


    }
}
