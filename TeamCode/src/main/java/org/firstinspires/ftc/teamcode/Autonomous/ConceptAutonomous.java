package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Library.ColorTools;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledClamp;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

@Autonomous (name = "ConceptAutonomous")
public class ConceptAutonomous extends LinearOpMode {

    private HardwareChassis robot;
    private ControlledDrive controlledDrive;
    private ControlledClamp controlledClamp;
    private ControlledLift controlledLift;
    private ColorTools colorTools;

    @Override
    public void runOpMode() {
        // --Initialize Robot--
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(hardwareMap, telemetry, () -> this.opModeIsActive());
        controlledClamp = new ControlledClamp(hardwareMap, telemetry, () -> this.opModeIsActive());
        controlledLift = new ControlledLift(hardwareMap, telemetry, () -> this.opModeIsActive());
        colorTools = new ColorTools();

        // --Main part--
        // Assuming we are Team Blue

        // Drive right, until we see a skystone -> vuforia

        // Center Skystone in Image

        // Drive forward
        controlledDrive.driveDistance(80, 0, 0.2, 5);

        // Grab skystone

        // dirve backwards until touchsensors are pressed
        controlledDrive.driveConditionally(-0.2, 0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        // sidewards till color_back sees $teamColor line
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);

        // forward till color_back sees $teamColor line
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);

        // raise arm, encoder based.
        controlledLift.raiseDistance(20, 0.2, 5);

        // drive forward until sensor sees $teamcolor
        controlledDrive.driveConditionally(0.2, 0, () -> colorTools.isBlue(robot.color_front));

        // hook platform with 2 servos

        // drive back, till touchsensors are pressed
        controlledDrive.driveConditionally(0.2,0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        // unhook the plate, my friend, with two servos

        // drive sidewards, till color_front sees $teamColor
        controlledDrive.driveConditionally(0,-0.2, () -> colorTools.isBlue(robot.color_front) == false);
    }
}
