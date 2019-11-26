package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.VuforiaNavigator;

@TeleOp(name="ConceptAutonomousVuforia")
public class ConceptAutonomousVuforia extends LinearOpMode {
    private VuforiaNavigator vuforiaNavigator;
    private HardwareChassis robot;

    @Override public void runOpMode() {
        this.robot = new HardwareChassis(hardwareMap);
        this.vuforiaNavigator = new VuforiaNavigator(hardwareMap, robot);




        //TODO: method grabSkystone() --> void, skystone greifen
    }
}
