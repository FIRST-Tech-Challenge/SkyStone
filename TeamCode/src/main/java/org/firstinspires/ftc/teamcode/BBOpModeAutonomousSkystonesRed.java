package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Auto-Skystones-Red", group="BB")
//@Disabled
public class BBOpModeAutonomousSkystonesRed extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBLinearSlide slide = new BBLinearSlide();
    private BBIntake intake = new BBIntake();
    private BBVision _vision = new BBVision();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);
        slide.init(hardwareMap, telemetry);
        intake.init(hardwareMap);
        _vision.setUp(telemetry, hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.moveForward(100,0.6);

        boolean foundStone = false;

        //look for the skystones for a period of time.
        List<Recognition> targets =  _vision.visionFeedback(telemetry);

        while(foundStone == false || runtime.seconds() < 10) {
            if (targets != null && targets.size() > 0) {
                //we found something!
                foundStone = true;

            } else {

                //strafe left
                //TODO: //strafe until we see a stone
                robot.strafe(0.3, 0.25);
            }
            targets =  _vision.visionFeedback(telemetry);
        }

        _vision.cleanUp();

        if(foundStone){
            //TODO: turn on intake?
            //TODO: we need to pick up stone - turn on intake? move arm?
            //TODO: turn towards left or right?
            //TODO: move robot and skystone into foundation area

            // extend arm
            slide.MoveUp();
            sleep(500);
            slide.StopArm();
            sleep(500);
            slide.SlideOut();
            sleep(500);
            slide.StopSlide();
            sleep(500);

            // grab skystone
            slide.Rotate();
            sleep(500);
            slide.MoveDown();
            sleep(500);
            slide.StopArm();
            sleep(500);
            slide.Grab();
            sleep(500);

            // move skystone towards robot and turn for intake
            slide.SlideIn();
            sleep(250);
            slide.StopSlide();
            sleep(500);
            slide.RotateReset();
            sleep(500);
            slide.Release();
            sleep(500);

            // draw back slide
            slide.MoveUp();
            sleep(500);
            slide.StopArm();
            sleep(500);
            slide.SlideIn();
            sleep(250);
            slide.StopSlide();
            sleep(500);

            // intake the skystone
            intake.Start();
            sleep(2000);
            intake.Stop();
        }else{
            //TODO: no stones found - park on line

        }


    }


}
