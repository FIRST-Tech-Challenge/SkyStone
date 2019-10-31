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
        runtime.reset();

        while(runtime.seconds() < 5) {
            if (targets != null && targets.size() > 0) {
                //we found something!
                telemetry.addLine("FOUND SOMETHING");
                telemetry.update();
                for(int count = 0; count < targets.size(); count++){
                    telemetry.addLine(targets.get(count).getLabel());
                    if(targets.get(count).getLabel() == "Skystone"){
                        telemetry.addLine("SKYSTONE FOUND");
                        telemetry.update();
                        foundStone = true;
                        robot.strafeForTime(0, 0);
                        break;
                    }
                }
                if(foundStone){
                    break;
                }

                robot.strafeForTime(0.5, 200);

            } else {
                telemetry.addLine("NOPE");
                telemetry.update();
                //strafe left
                robot.strafeForTime(0.5, 200);
            }
            targets =  _vision.visionFeedback(telemetry);
        }

        _vision.cleanUp();
        robot.strafeForTime(0, 0);
        if(foundStone){



            slide.MoveUp();

            sleep(500);
            slide.StopArm();

            slide.SlideOut();
            sleep(3000);
            slide.StopSlide();


            slide.RotateReset();
            sleep(250);
            slide.ReLevel(1);
            sleep(2400);
            slide.LevelStop();
            sleep(300);


            sleep(300);
            slide.MoveDown(0.1);
            sleep(1500);
            slide.StopArm();
            slide.Level(1);
            sleep(1500);
            slide.Grab();
            sleep(500);

            robot.turnRight(105,0.6);
            robot.moveForward(140, 0.6);

        }else{

            robot.turnRight(110, 0.70);
            robot.moveForward(150,0.6);
        }


    }


}
