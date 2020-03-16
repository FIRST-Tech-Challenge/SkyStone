package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class foundationTowerMove extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        telemetry.addData("Status", "initialized");
        joules.TapeMeasurePush();

        waitForStart();



        clearTimer(1);
        while (opModeIsActive() && getMilliSeconds(1)< 100){
            joules.DriveBackward(0.5);
        }
        joules.Stop();

        clearTimer(1);
        while (opModeIsActive() && getMilliSeconds(1)< 2000){
            joules.FoundationDrop();
        }
        joules.Stop();


        clearTimer(1);
        while (opModeIsActive() && getMilliSeconds(1)< 2000){
            joules.DaffyUp();
        }
        joules.Stop();


    }
}
