package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Library.ColorTools;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledArm;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledClamp;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

@Autonomous (name = "ConceptAutonomous")
public class ConceptAutonomous extends LinearOpMode {

    private HardwareChassis robot;
    private ControlledDrive controlledDrive;
    private ControlledClamp controlledClamp;
    private ControlledArm controlledArm;
    private ColorTools colorTools;

    @Override
    public void runOpMode() {
        // --Initialize Robot--
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(hardwareMap, telemetry);
        controlledClamp = new ControlledClamp(hardwareMap, telemetry);
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

        // forward till color_front sees $teamColor line
        controlledDrive.driveConditionally(0.2,0, () -> colorTools.isBlue(robot.color_back) == false);

        // forward till color_front sees $teamColor line
        controlledDrive.driveConditionally(0.2,0, () -> colorTools.isBlue(robot.color_front) == false);

        // drive right, till color_back sees $teamColor
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);

        // drive back, till color_back not sees $teamColor
        controlledDrive.driveConditionally(-0.2,0, () -> colorTools.isBlue(robot.color_back) == true);

        // raise arm, encoder based.
        controlledArm.raiseDistance(20, 0.2, 5);


        // drive right, till color_back sees $teamColor
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);

        // open clamp, encoder based
        controlledClamp.openDistance(20, 0.2, 5);

        // hook platform

        // drive left, till touchsensor is pressed
        controlledDrive.driveConditionally(0,-0.2, () -> robot.touch_left.getState() == true);

        // unhook the plate, my friend

        // drive back, till color_back sees $teamColor
        controlledDrive.driveConditionally(-0.2,0, () -> colorTools.isBlue(robot.color_back) == false);
    }
}
