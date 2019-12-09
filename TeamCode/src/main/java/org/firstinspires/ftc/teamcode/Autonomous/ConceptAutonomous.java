package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Library.ColorTools;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledArm;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

/*
//auskommentiert weil error
@Autonomous (name = "ConceptAutonomous")

public class ConceptAutonomous extends LinearOpMode {

    HardwareChassis robot;
    ControlledDrive controlledDrive;
    //ControlledClamp controlledClamp;
    ControlledArm controlledArm;
    ColorTools colorTools;

    @Override
    public void runOpMode() {
        // --Initialize Robot--
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(hardwareMap);
        //controlledClamp = new ControlledClamp(hardwareMap, telemetry);
        controlledArm = new ControlledArm(hardwareMap, telemetry);
        colorTools = new ColorTools();

        // --Main part--
        // Assuming we are Team Blue

        // Drive right, until we see a skystone -> vuforia

        // Center Skystone in Image

        // Drive forward
        controlledDrive.driveDistance(80, 0, 0.2, 5);

        // Grab skystone

        // turn 90°
        controlledDrive.rotate(90, 0.2, 5);

        // drive left, until touchsensors are pressed
        controlledDrive.driveConditionally(0,-0.2, () -> robot.touch_left.getState() == true);

        // forward till color_left sees $teamColor line
        controlledDrive.driveConditionally(0.2,0, () -> colorTools.isBlue(robot.color_right) == false);

        // forward till color_left sees $teamColor line
        controlledDrive.driveConditionally(0.2,0, () -> colorTools.isBlue(robot.color_left) == false);

        // drive right, till color_right sees $teamColor
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_right) == false);

        // drive back, till color_right not sees $teamColor
        controlledDrive.driveConditionally(-0.2,0, () -> colorTools.isBlue(robot.color_right) == true);

        // raise arm, encoder based.
        controlledArm.raiseDistance(20, 0.2, 5);


        // drive right, till color_right sees $teamColor
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_right) == false);

        // open clamp, encoder based
        //controlledClamp.openDistance(20, 0.2, 5);

        // hook platform

        // drive left, till touchsensor is pressed
        controlledDrive.driveConditionally(0,-0.2, () -> robot.touch_left.getState() == true);

        // unhook the plate, my friend

        // drive back, till color_right sees $teamColor
        controlledDrive.driveConditionally(-0.2,0, () -> colorTools.isBlue(robot.color_right) == false);
    }

}

 */

