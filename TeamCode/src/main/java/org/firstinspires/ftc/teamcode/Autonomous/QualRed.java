package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Qual Red", group = "basic")
public class QualRed extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            rob.driveTrainMovement(.3, Crane.movements.backward);
            sleep(1500);
            rob.foundationServo1.setPosition(.6);
            rob.foundationServo2.setPosition(0);
            sleep(1000);
            rob.driveTrainMovement(.6, Crane.movements.right);
            sleep(750);
            rob.stopDrivetrain();
            sleep(500);
            rob.stopDrivetrain();
            sleep(500);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(1000);
            rob.driveTrainMovement(.6, Crane.movements.cw);
            sleep(1750);
            rob.driveTrainMovement(.6, Crane.movements.backward);
            sleep(1500);
            rob.foundationServo1.setPosition(0);
            rob.foundationServo2.setPosition(.7);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(500);
            rob.driveTrainMovement(.6, Crane.movements.ccw);
            sleep(500);
            rob.driveTrainMovement(.6, Crane.movements.left);
            sleep(1000);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(2000);
        }
//hi

    }
}
