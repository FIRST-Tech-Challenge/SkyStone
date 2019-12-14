package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="useless", group = "basic")
public class Foundation extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation, Crane.setupType.claw);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){

            rob.driveTrainMovement(0.5, Crane.movements.forward);
            sleep(1400);
            rob.driveTrainMovement(0.5, Crane.movements.right);
            sleep(2900);
            rob.stopDrivetrain();
            //rob.foundationServo.setPosition(0);
            sleep(2000);
            rob.driveTrainMovement(1, Crane.movements.left);
            sleep(2550);
            //rob.foundationServo.setPosition(0.5);
            rob.driveTrainMovement(1, Crane.movements.forward);
            sleep(3000);
        }


    }
}
