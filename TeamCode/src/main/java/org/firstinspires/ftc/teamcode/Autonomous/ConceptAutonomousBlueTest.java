package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.VuforiaNavigator;

@Autonomous(name="ConceptAutonomousBlueTest")
public class ConceptAutonomousBlueTest extends LinearOpMode {
    private VuforiaNavigator vuforiaNavigator;
    private HardwareChassis robot;
    private ControlledDrive controlledDrive;
    private GeneralTools generalTools;
    private ColorTools colorTools;

    @Override public void runOpMode() {
        this.robot = new HardwareChassis(hardwareMap);
        this.vuforiaNavigator = new VuforiaNavigator(hardwareMap, robot, telemetry, () -> opModeIsActive());
        this.controlledDrive= new ControlledDrive(hardwareMap, telemetry, () -> opModeIsActive());
        this.generalTools = new GeneralTools(this, robot);
        this.colorTools = new ColorTools();

        int direction = 1; // LEFT=1, RIGHT=-1


        //drive method, speed forward, speed sideways
        //controlledDrive.driveConditionally(0.5, 0, () -> true);

        //controlledDrive.driveConditionally(0,0.2*direction, () -> (!vuforiaNavigator.skystoneFound() && opModeIsActive()));
        //vuforiaNavigator.navigateToSkystone(0.5, 0.5);
        controlledDrive.driveDistance(80, 0, 0.2, 5);

        // TODO: Arm runter
        generalTools.grabSkysstone();
        /*

        controlledDrive.driveConditionally(-0.5,0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        controlledDrive.driveConditionally(0,-0.2*direction, () -> colorTools.isRed(robot.color_back) == false);
        controlledDrive.driveDistance(0, -5*direction, 0.2, 0);
        controlledDrive.driveConditionally(0,-0.2*direction, () -> colorTools.isRed(robot.color_back) == false);
        // we are now in the corner

        // TODO: arm hoch
        controlledDrive.driveConditionally(0.5,0, () -> colorTools.isRed(robot.color_front) == false);
        // TODO: arm runter

        controlledDrive.driveConditionally(-0.5,0, () -> robot.touch_left.getState() == true && robot.touch_right.getState() == true);

        generalTools.openClamp();
        // TODO: arm hoch

        controlledDrive.driveDistance(0, 20*direction, 0.2, 0);

        // TODO: arm runter

        controlledDrive.driveConditionally(0,0.5*direction, () -> colorTools.isRed(robot.color_front) == false);

         */
    }
}
