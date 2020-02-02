package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@Autonomous(name="Red_Load_Skystone", group="game")
//@Disabled
public class Red_Load_Skystone extends LinearOpMode {

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

         /*  robot.driveTrain.encoderDrive(1, -0.5);
           robot.driveTrain.gyroRotate(90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -40);
           robot.driveTrain.gyroRotate(-90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -10);
           robot.driveTrain.encoderDrive(1, 10); */

        // Move the robot to the wall
        robot.driveTrain.moveBackward(1,0.5);

        // Pause
        sleep(500);

        // Move the robot along to the wall
        robot.driveTrain.crabRight(0.8);

        // Pause
        sleep(500);

        // Re-position the robot

        robot.driveTrain.moveBackward(0.5,1);

        // Move the robot backwards along the skystones
        robot.driveTrain.moveForward(4,0.2);

        // Stop
        stop();
        /*
        runtime.reset();

        while(runtime.seconds() < 60){
            robot.driveTrain.moveForward(0.5, 1);
            if(robot.skystoneFinder.canSeeSkystone()){
                robot.driveTrain.moveForward(.5, .25);

            }
        }

    //With Phone Camera Mounted on Side Pannel

        //robot.logger.logInfo("runOpMode", "Angles: 1:%f", angle1);

        //  with robot in square touching blue bridge and wall

        // with robot orientated with intake towards wall

        // move back 1 tile
        robot.driveTrain.moveBackward(.1, .5);
        // rotate 45 degrees away from bridge
        robot.driveTrain.gyroRotate(-45, .5);
        // stop do vision
            //Get x-y-z coordinates from vuforia of skystone

            //If no coordinates, move to set position

            //If corrdinate is found (found x and y position), then find the x-y-z position of skystone

            //If x - y matches (23< x <25)recorded position for skystone



        // if skystone in position 1 (to be determined what position 1 is in coordinates)
        //    execute sequence picking up from block 1 position
        // if skystone in position 2
        //    execute sequence picking up from block 2 position
        // if skystone in position 3
        //    execute sequence picking up from block 2 position


        // after block pickup, return to set position which is tile adjacent to
        //   alliance and neutral bridge

*/
        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
