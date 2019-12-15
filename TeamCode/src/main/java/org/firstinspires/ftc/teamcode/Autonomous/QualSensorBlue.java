package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Qual Sensor Blue", group = "basic")
public class QualSensorBlue extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation, Crane.setupType.ultrasoinc);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){

            while(rob.front.getDistance(DistanceUnit.CM) < 10){
                rob.driveTrainMovement(0.3, Crane.movements.backward);
            }
            sleep(500);
            rob.foundationServo1.setPosition(.6);
            rob.foundationServo2.setPosition(0);
            //turn 180
            rob.driveTrainMovement(.2, Crane.movements.forward);
            sleep(1000);
            while(rob.right.getDistance(DistanceUnit.CM) > 10){
                rob.driveTrainMovement(0.3, Crane.movements.right);
            }
            rob.foundationServo1.setPosition(0);
            rob.foundationServo2.setPosition(.7);
            //90 CW
            while(!(rob.color.red() == 180)) {
                rob.driveTrainMovement(0.3, Crane.movements.forward);
            }



            /*
            rob.driveTrainMovement(.3, Crane.movements.backward);
            sleep(1500);
            rob.foundationServo1.setPosition(.6);
            rob.foundationServo2.setPosition(0);
            sleep(1000);
            rob.driveTrainMovement(.6, Crane.movements.left);
            sleep(750);
            rob.stopDrivetrain();
            sleep(500);
            rob.stopDrivetrain();
            sleep(500);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(1000);
            rob.driveTrainMovement(.6, Crane.movements.ccw);
            sleep(1750);
            rob.driveTrainMovement(.6, Crane.movements.backward);
            sleep(1500);
            rob.foundationServo1.setPosition(0);
            rob.foundationServo2.setPosition(.7);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(500);
            rob.driveTrainMovement(.6, Crane.movements.cw);
            sleep(500);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(2000);
             */
        }


    }
}
