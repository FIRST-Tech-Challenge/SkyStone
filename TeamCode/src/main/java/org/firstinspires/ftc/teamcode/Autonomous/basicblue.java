package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;
//hi
@Autonomous(name="basicblue", group = "basic")
public class basicblue extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){

            rob.driveTrainMovement(.75, Crane.movements.forward);
            sleep(750);
            rob.stopDrivetrain();

            sleep(1000);

            rob.driveTrainMovement(.75, Crane.movements.right);
            sleep(800);
            rob.stopDrivetrain();

            sleep(1000);

            //rob.driveTrainMovement(10, Crane.movements.ccw);
            //sleep(300);
            //rob.stopDrivetrain();

            //sleep(1000);

            rob.driveTrainMovement(.75, Crane.movements.right);
            sleep(1300);
            rob.stopDrivetrain();

            sleep(1000);


            //rob.driveTrainMovement(10, Crane.movements.forward);
            //sleep(100);
            //rob.stopDrivetrain();

            //sleep(1000);

            //rob.driveTrainMovement(10, Crane.movements.cw);
            //sleep(100);
            //rob.stopDrivetrain();

            //sleep(1000);

            //rob.foundationServo.setPosition(0.1);

            sleep(1000);

            rob.driveTrainMovement(.75, Crane.movements.left);
            sleep(1300);
            rob.stopDrivetrain();

            //rob.driveTrainMovement(10, Crane.movements.forward);
            //sleep(1000);
            //rob.stopDrivetrain();

            rob.driveTrainMovement(.75, Crane.movements.left);
            sleep(1500);
            rob.stopDrivetrain();

            rob.driveTrainMovement(.75, Crane.movements.forward);
            sleep(1000);
            rob.stopDrivetrain();

            rob.driveTrainMovement(.75, Crane.movements.left);
            sleep(1100);
            rob.stopDrivetrain();

            //rob.foundationServo.setPosition(0.5);

            sleep(1000);

            rob.driveTrainMovement( .75, Crane.movements.backward);
            sleep(2125);
            rob.stopDrivetrain();

        }


    }
}
