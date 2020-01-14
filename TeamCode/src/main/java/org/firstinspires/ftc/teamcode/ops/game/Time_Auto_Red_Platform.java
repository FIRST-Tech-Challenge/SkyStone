package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@Autonomous(name="Time_Auto_Red_Platform", group="game")
//@Disabled
public class Time_Auto_Red_Platform extends LinearOpMode {

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

        //Move the robot down the field twoards the build platform
        robot.driveTrain.moveBackward(.95, -.75);

        //Rotate the back of the robot twoards the build platform
        robot.driveTrain.gyroRotate(-85, .75, true, false);

        //Move the robot so that it is touching the build platform
        robot.driveTrain.moveBackward(0.7, -0.5);

        //Pause to let the robot stop moving
        robot.driveTrain.pause(2 );

        //Move the servos down to grapple the build platform
        //robot.grapple.servoMoveDown();
        robot.grapple.servo2MoveDown();

        //Pause
        robot.driveTrain.pause(2);

        //Pull the platform twoards the build zone
        robot.driveTrain.moveForward(1.25, -0.5);

        //Move because the robot can not fine adjust to make the gyro happy with the platform in tow
        //robot.driveTrain.move(1, -1, 1);

        robot.driveTrain.gyroRotate(95, .5);

        //Move the servos up to release the platform
        //robot.grapple.servoMoveUp();
        //robot.grapple.servo2MoveUp();

        //Push the build platform to the wall to score it
        robot.driveTrain.moveBackward(.5, .75);

        //Pause to let the robot stop moving
        robot.driveTrain.pause(.25);

        //Move the robot to park under the skybridge
        robot.driveTrain.moveForward(0.9,  .75);

        robot.driveTrain.pause(5 );
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
