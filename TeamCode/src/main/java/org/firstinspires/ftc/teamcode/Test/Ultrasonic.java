package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="ultrasonic", group = "basic")
public class Ultrasonic extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.ultrasoinc);
        telemetry.addLine("Start!");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic front", rob.front.rawUltrasonic());
            telemetry.addData("raw optical front", rob.front.rawOptical());
            telemetry.addData("cm optical front", "%.2f cm", rob.front.cmOptical());
            telemetry.addData("cm front", "%.2f cm", rob.front.getDistance(DistanceUnit.CM));

            telemetry.addData("raw ultrasonic back", rob.back.rawUltrasonic());
            telemetry.addData("raw optical back", rob.back.rawOptical());
            telemetry.addData("cm optical back", "%.2f cm", rob.back.cmOptical());
            telemetry.addData("cm back", "%.2f cm", rob.back.getDistance(DistanceUnit.CM));

            telemetry.update();
        }


    }
}
