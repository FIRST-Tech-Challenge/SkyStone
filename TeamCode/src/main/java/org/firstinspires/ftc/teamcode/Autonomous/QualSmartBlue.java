package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Qual Smart Blue", group = "basic")
public class QualSmartBlue extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation, Crane.setupType.imu, Crane.setupType.ultrasoinc);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            double robOrient = 90;
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
            rob.stopDrivetrain();


            double dist = 0;
            double currdir = 0;
            telemetry.addData("Angle:", rob.getDirection() - robOrient);
            telemetry.update();

            double correctedDist = 0;
            do{
                rob.driveTrainMovement(0.3, Crane.movements.forward);

                dist= rob.back.getDistance(DistanceUnit.INCH);
                currdir = rob.getDirection();
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.addData("Angle:", currdir - robOrient);
                telemetry.addData("Corrected:", correctedDist);
                telemetry.update();

                correctedDist = dist * Math.cos(Math.toRadians(currdir - robOrient));

            }
            while(correctedDist < 64.5 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

            sleep(2000);
            sleep(40000);
        }


    }
}
