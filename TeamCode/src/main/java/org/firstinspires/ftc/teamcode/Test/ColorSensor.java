package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

@Autonomous(name="Color Sensor", group = "basic")
public class ColorSensor extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.ultrasoinc);
        telemetry.addLine("Start!");
        telemetry.update();

        while (opModeIsActive()) {

            rob.color.enableLed(true);

            telemetry.addData("Clear", rob.color.alpha());
            telemetry.addData("Red  ", rob.color.red());
            telemetry.addData("Green", rob.color.green());
            telemetry.addData("Blue ", rob.color.blue());

            telemetry.update();
        }


    }
}
