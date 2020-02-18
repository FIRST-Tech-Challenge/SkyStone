package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class RandomStoneBlue extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ColorSensor colorSensor;


        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        colorSensor = hardwareMap.colorSensor.get("colour");

        telemetry.addData("Status", "initialized");

        int bluetapeval = 27;

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
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 350));
        joules.Stop();

        joules.StrafeLeft(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 2000));
        joules.Stop();

        joules.DaffyUp();
        sleep(100);
        joules.DaffyStop();

        clearTimer(1);
        while (opModeIsActive() && getSeconds(1) < 1000 && colorSensor.blue() < bluetapeval) {
                    joules.StrafeRight(0.3);
                    telemetry.addData("Seconds", getSeconds(1));
                    telemetry.addData("blue", colorSensor.blue());
            }

            joules.Stop();
        joules.DaffyGrab();
        joules.DriveForward(0.4);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(), 600));
        joules.DaffyStop();
        joules.Stop();






    }

}
