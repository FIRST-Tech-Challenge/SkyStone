package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;

@Autonomous

public class CBackwardUpPark extends AutoOpMode {
    private VoltageSensor ExpansionHub2_VoltageSensor;
    public void runOp() throws InterruptedException {
        Joules joules = new Joules();
        ExpansionHub2_VoltageSensor =  hardwareMap.voltageSensor.get("Expansion Hub 2");

        telemetry.addData("Status", "initialized");
        joules.TapeMeasurePush();

        waitForStart();

        joules.StrafeLeft(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),1050));
        joules.Stop();


        sleep(100);

        joules.DriveBackward(0.5);
        sleep(joules.getSeconds(ExpansionHub2_VoltageSensor.getVoltage(),1000));
        joules.Stop();


    }

}
