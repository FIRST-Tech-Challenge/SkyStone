package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class OnFoundBothRed extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();

        ColorSensor colorSensorLeft;
        ColorSensor colorSensorRight;
        ColorSensor colorSensordown;

        DistanceSensor DistanceSensorLeft;
        DistanceSensor DistanceSensorRight;

        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        colorSensorLeft = hardwareMap.colorSensor.get("colourLeft");

        colorSensorRight = hardwareMap.colorSensor.get("colourRight");

        colorSensordown = hardwareMap.colorSensor.get("colourDown");
        telemetry.addData("Status", "initialized");

        int redtapeval = 27;

        waitForStart();

        joules.DaffyUp();
        sleep( 2000);
        joules.DaffyStop();

        joules.DriveForward(0.4);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 650));
        joules.Stop();

        joules.DriveForward(0.2);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 600));
        joules.Stop();

        joules.DaffyGrab();
        sleep(2000);

        joules.SlidesUp();
        sleep(100);
        joules.SlidesStop();

        joules.DriveBackward(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 600));
        joules.StrafeRight(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 2000));
        joules.Stop();
        

        joules.SlidesUp();
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),800));
        joules.SlidesStop();


        joules.StrafeRight(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1000));
        joules.Stop();
        sleep(500);

        clearTimer(1);
        while (opModeIsActive() && getSeconds(1) < 2000 && colorSensorLeft.red() < redtapeval) {
                    joules.DriveForward(0.1);
                    telemetry.addData("Seconds", getSeconds(1));
                    telemetry.addData("blue", colorSensorLeft.red());
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
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 900));
        joules.Stop();

        joules.FoundationDrop();
        sleep(2000);

        joules.DaffyUp();
        sleep(1000);
        joules.Stop();

        joules.DriveBackward(0.3);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 1200));
        joules.Stop();





    }

}
