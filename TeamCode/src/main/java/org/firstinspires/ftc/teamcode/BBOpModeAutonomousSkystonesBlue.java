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
    private BBIntake intake = new BBIntake();
    private BBVision _vision = new BBVision();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);

        intake.init(hardwareMap);
        _vision.setUp(telemetry, hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



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

                        break;
                    }
                }
                if(foundStone){
                    break;
                }


            } else {
                telemetry.addLine("NOPE");
                telemetry.update();
                //strafe left


            }
            targets =  _vision.visionFeedback(telemetry);
        }

        _vision.cleanUp();

        if(foundStone){



        }else{

            robot.turnLeft(110, 0.70);
            if(runtime.seconds() > 10){
                //robot.moveForward(190, 0.6);
            }else {
                //robot.moveForward(150, 0.6);
            }
        }


    }


}
