package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class OnFoundationStone extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;


        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        colorSensor = hardwareMap.colorSensor.get("colour");

        telemetry.addData("Status", "initialized");

        int bluetapeval = 27;

        waitForStart();

        joules.DaffyGrab();
        sleep(1000);

        joules.SlidesUp();
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),800));
        joules.SlidesStop();


        joules.StrafeRight(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1000));
        joules.Stop();
        sleep(500);

        clearTimer(1);
        while (opModeIsActive() && getSeconds(1) < 2000 && colorSensor.blue() < bluetapeval) {
                    joules.DriveForward(0.1);
                    telemetry.addData("Seconds", getSeconds(1));
                    telemetry.addData("blue", colorSensor.blue());
        }
        joules.Stop();

        joules.SlidesDown();
        joules.FoundationGrab();
        sleep(100);
        joules.SlidesStop();
        sleep(1500);

        joules.DriveBackward(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 600));
        joules.Stop();

        joules.StrafeLeft(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 600));
        joules.Stop();

        joules.TurnLeft(0.5);
        sleep(750);
        joules.Stop();

        joules.DriveForward(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 800));
        joules.Stop();

        joules.StrafeLeft(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1000));
        joules.Stop();





    }

}
