package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

/*
@Autonomous(name="ConceptAutonomousBlueTest")
public class ConceptAutonomousBlueTest extends LinearOpMode {
    private HardwareChassis robot;
    private GeneralTools generalTools;
    private ColorTools colorTools;

    @Override public void runOpMode() {
        this.robot = new HardwareChassis(hardwareMap);
        this.generalTools = new GeneralTools(this, robot);
        this.colorTools = new ColorTools();

        int direction = 1; // LEFT=1, RIGHT=-1


        //drive method, speed forward, speed sideways
        //controlledDriveOld.driveConditionally(0.5, 0, () -> true);

        //controlledDriveOld.driveConditionally(0,0.2*direction, () -> (!vuforiaNavigator.skystoneFound() && opModeIsActive()));
        //vuforiaNavigator.navigateToSkystone(0.5, 0.5);
        ControlledDrive driveForward = new ControlledDrive(robot, telemetry);

        driveForward.start(80, 0, 0.2);
        while(!driveForward.endReached() && !isStopRequested()) {}
        driveForward.stop();

        // TODO: Arm runter
        generalTools.closeClamp();
        /*

        controlledDriveOld.driveConditionally(-0.5,0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        controlledDriveOld.driveConditionally(0,-0.2*direction, () -> colorTools.isRed(robot.color_back) == false);
        controlledDriveOld.driveDistance(0, -5*direction, 0.2, 0);
        controlledDriveOld.driveConditionally(0,-0.2*direction, () -> colorTools.isRed(robot.color_back) == false);
        // we are now in the corner

        // TODO: arm hoch
        controlledDriveOld.driveConditionally(0.5,0, () -> colorTools.isRed(robot.color_front) == false);
        // TODO: arm runter

        controlledDriveOld.driveConditionally(-0.5,0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        generalTools.openClamp();
        // TODO: arm hoch

        controlledDriveOld.driveDistance(0, 20*direction, 0.2, 0);

        // TODO: arm runter

        controlledDriveOld.driveConditionally(0,0.5*direction, () -> colorTools.isRed(robot.color_front) == false);


    }
}

*/