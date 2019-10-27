package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Auto-Skystones-Blue", group="BB")
//@Disabled
public class BBOpModeAutonomousSkystonesBlue extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBVision _vision = new BBVision();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);
        _vision.setUp(telemetry,hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

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
        }


    }


}
