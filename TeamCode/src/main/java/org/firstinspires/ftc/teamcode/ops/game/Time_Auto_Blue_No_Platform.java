package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@Autonomous(name="Time_Auto_Blue_No_Platform", group="game")
//@Disabled
public class Time_Auto_Blue_No_Platform extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.gyroNavigator.init();
        //robot.gyroNavigator2.init();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();

        double angle1 = robot.gyroNavigator.getAngle();
        //   double angle2 = robot.gyroNavigator2.getAngle();

        //---BEGIN AUTONOMOUS---BEGIN AUTONOMOUS---BEGIN AUTONOMOUS---BEGIN AUTONOMOUS---//

        //With Phone Camera Mounted on Side Pannel

        //robot.logger.logInfo("runOpMode", "Angles: 1:%f", angle1);

        //  with robot in square touching blue bridge and wall

        // with robot orientated with intake towards wall

        // move back 1 tile

        robot.driveTrain.moveBackward(.1, .5);

        // rotate 45 degrees away from the Skybridge
        robot.driveTrain.gyroRotate(-45, .5);

        // stop do vision

        //Get x-y-z coordinates from vuforia of skystone

        //If no coordinates, move to set position

        //If corrdinate is found (found x and y position), then find the x-y-z position of skystone

        //If x - y matches (23< x <25)recorded position for skystone



        // if skystone in position 1 (to be determined what position 1 is in coordinates)
            robot.driveTrain.gyroRotate(135, .5);
        // if skystone in position 2
        //    execute sequence picking up from block 2 position
        // if skystone in position 3
        //    execute sequence picking up from block 2 position


        // after block pickup, return to set position which is tile adjacent to
        //   alliance and neutral bridge

        //SKYSTONE ANALYSIS AND PICKUP PROCEDURE

        //Move the robot down the field towards the build platform
        robot.driveTrain.moveBackward(.30, .75);

        //Rotate the back of the robot towards the build platform
        robot.driveTrain.gyroRotate(85, .75, true, false);

        //Move the robot so that it is touching the build platform
        robot.driveTrain.moveBackward(0.6, 0.5);

        //Pause to let the robot stop moving
        robot.driveTrain.pause(.25);

        //Move the servos down to grapple the build platform
        robot.ramp.ramp2Down();
        robot.ramp.rampDown();

        //Pause to let stone fall out
        robot.driveTrain.pause(.25);

        //Move ramp back up
        robot.ramp.ramp2Up();
        robot.ramp.rampUp();

        //Move the robot forward so that it is next to the parking spot
        robot.driveTrain.moveForward(0.6, 0.5);

        //Turn the robot so that it can drive under the bridge
        robot.driveTrain.gyroRotate(85, .5, true, false);

        //Park under the SkyBridge
        robot.driveTrain.moveForward(.25, 0.5);

        robot.driveTrain.pause(.25);

        runtime.reset();

        while(runtime.seconds() < 60){
            robot.driveTrain.moveForward(0.5, 1);
            if(robot.skystoneFinder.canSeeSkystone()){
                robot.driveTrain.moveForward(.5, 1);

            }
        }

        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
