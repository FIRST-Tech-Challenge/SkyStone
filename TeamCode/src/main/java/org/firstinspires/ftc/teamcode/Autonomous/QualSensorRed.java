package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Qual Sensor Red", group = "AA")
public class QualSensorRed extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int n =1;
        setup(runtime, Crane.setupType.drive, Crane.setupType.foundation, Crane.setupType.ultrasoinc, Crane.setupType.imu);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            double dist = 0;
            do{
                rob.driveTrainMovement(0.3, Crane.movements.backward);

                dist= rob.back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist > 20 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

            do{
                rob.driveTrainMovement(0.1, Crane.movements.backward);

                dist= rob.back.getDistance(DistanceUnit.CM);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist > 5 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

            sleep(500);
            rob.foundationServo1.setPosition(.6);
            rob.foundationServo2.setPosition(0);
            sleep(500);

            telemetry.addData("cm front", "%.2f cm", rob.front.getDistance(DistanceUnit.CM));

            telemetry.update();

            //turn 180

            rob.driveTrainMovement(.2, Crane.movements.forward);
            sleep(1000);
            do{
                rob.driveTrainMovement(0.4, Crane.movements.right);

                dist= rob.left.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist < 42 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

            rob.turn(90, Crane.turnside.cw, 0.6, Crane.axis.center);
            rob.driveTrainMovement(.6, Crane.movements.backward);
            sleep(2000);
            rob.foundationServo1.setPosition(0);
            rob.foundationServo2.setPosition(.7);
            rob.driveTrainMovement(.3, Crane.movements.forward);
            sleep(500);
            rob.absturn(90, Crane.turnside.ccw, 0.6, Crane.axis.center);
            sleep(800);


            do{
                rob.driveTrainMovement(0.6, Crane.movements.left);

                dist= rob.right.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist > 6 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
            sleep(800);
            do{
                rob.driveTrainMovement(0.35, Crane.movements.forward);

                dist= rob.back.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist < 48 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());
            do{
                rob.driveTrainMovement(0.15, Crane.movements.forward);

                dist= rob.back.getDistance(DistanceUnit.INCH);
                telemetry.addData("cm front", "%.2f cm", dist);
                telemetry.update();

            }
            while(dist < 60 || Double.compare(dist, Double.NaN) == 0 && opModeIsActive());

        }


    }
}
