package org.firstinspires.ftc.teamcode.Autonomous;

/*
//auskommentiert weil error
@Autonomous (name = "ConceptAutonomous")

public class ConceptAutonomous extends LinearOpMode {

    HardwareChassis robot;
    ControlledDriveOld controlledDrive;
    //ControlledClamp controlledClamp;
    ControlledLift controlledLift;
    ColorTools colorTools;

    @Override
    public void runOpMode() {
        // --Initialize Robot--
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDriveOld(hardwareMap, telemetry, () -> this.opModeIsActive());
        controlledLift = new ControlledLift(hardwareMap, telemetry, () -> this.opModeIsActive());
        //controlledClamp = new ControlledClamp(hardwareMap, telemetry, () -> this.opModeIsActive());
        colorTools = new ColorTools();
        /*

        // --Main part--
        // Assuming we are Team Blue

        // Drive right, until we see a skystone -> vuforia

        // Center Skystone in Image

        // Drive forward
        controlledDrive.driveDistance(80, 0, 0.2, 5);

        // Grab skystone

<<<<<<< HEAD
        // dirve backwards until touchsensors are pressed
        controlledDrive.driveConditionally(-0.2, 0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        // sidewards till color_back sees $teamColor line
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);

        // forward till color_back sees $teamColor line
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);
=======
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
>>>>>>> hardware_tests

        // raise arm, encoder based.
        controlledLift.raiseDistance(20, 0.2, 5);

        // drive forward until sensor sees $teamcolor
        controlledDrive.driveConditionally(0.2, 0, () -> colorTools.isBlue(robot.color_front));

<<<<<<< HEAD
        // hook platform with 2 servos
=======
        // drive right, till color_back sees $teamColor
        controlledDrive.driveConditionally(0,0.2, () -> colorTools.isBlue(robot.color_back) == false);
>>>>>>> hardware_tests

        // open clamp, encoder based
        //controlledClamp.openDistance(20, 0.2, 5);

        // unhook the plate, my friend, with two servos

<<<<<<< HEAD
        // drive sidewards, till color_front sees $teamColor
        controlledDrive.driveConditionally(0,-0.2, () -> colorTools.isBlue(robot.color_front) == false);
=======
        // drive left, till touchsensor is pressed
        controlledDrive.driveConditionally(0,-0.2, () -> robot.touch_left.getState() == true);

        // unhook the plate, my friend

        // drive back, till color_back sees $teamColor
        controlledDrive.driveConditionally(-0.2,0, () -> colorTools.isBlue(robot.color_back) == false);
>>>>>>> hardware_tests
    }

}

 */

