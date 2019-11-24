package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Straight Auton", group = "basic")
public class straight extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            rob.driveTrainMovement(0, Crane.movements.forward);
            sleep(1000);
            rob.driveTrainMovement(0.0001, Crane.movements.forward);
            sleep(2500);
            rob.driveTrainMovement(0.001, Crane.movements.forward);
            sleep(2500);
            rob.driveTrainMovement(0.01, Crane.movements.forward);
            sleep(2500);
            rob.driveTrainMovement(0.1, Crane.movements.forward);
            sleep(2500);
            rob.stopDrivetrain();



        }


    }
}
