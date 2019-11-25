package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

import java.util.List;

@Autonomous(name="Auto-Skystones-Red", group="BB")
//@Disabled
public class BBOpModeAutonomousSkystonesRed extends LinearOpMode
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

        int movesForward = 0;

        boolean foundStone = false;


        robot.RobotMoveX(new Waypoint(25, 0, 0 ), 0.2);
        robot.Stop();

        //look for the skystones for a period of time.
        List<Recognition> targets =  _vision.visionFeedback(telemetry);
        runtime.reset();



        Recognition foundTarget = null;

        while(runtime.seconds() < 15 && this.opModeIsActive() && foundStone == false && movesForward < 15) {
            if (targets != null && targets.size() > 0) {
                //we found something!

                for(int count = 0; count < targets.size(); count++){
                    if(targets.get(count).getLabel() == "Skystone")
                    {
                        foundTarget = targets.get(count);
                        telemetry.addLine("SKYSTONE FOUND");
                        telemetry.addData("Pos", foundTarget.getLeft());
                        telemetry.addData("Right", foundTarget.getRight());
                        telemetry.addData("Left", foundTarget.getLeft());
                        telemetry.addData("Left", foundTarget.getLabel());
                        telemetry.update();
                        sleep(1000);
                        foundStone = true;
                        robot.Stop();
                        break;
                    }
                }

                if(foundStone){
                    break;
                }
                telemetry.addLine("Moving forward now");
                telemetry.update();
                robot.RobotMoveY(new Waypoint(0, 10, 0 ), 0.2);
                robot.Stop();
                TimeElapsedPause(500);
                movesForward++;

            } else {

                //move left (i.e. forward)
                robot.RobotMoveY(new Waypoint(0, 5, 0 ), 0.2);
                robot.Stop();
                TimeElapsedPause(1000);
                movesForward++;


            }
            targets =  _vision.visionFeedback(telemetry);
        }



        if(foundStone){

            //we need to move to the stone (ie. left or right)
            telemetry.addLine("Moving towards the skystones");
            //use the left / right position to determine how far to move
            telemetry.addData("L", foundTarget.getLeft());
            telemetry.addData("R", foundTarget.getRight());
            telemetry.addData("Label", foundTarget.getLabel());

            telemetry.addData("Angle", foundTarget.estimateAngleToObject(AngleUnit.DEGREES));
            double foundAngle = foundTarget.estimateAngleToObject(AngleUnit.DEGREES);
            telemetry.update();

           // sleep(2000);


            while(this.opModeIsActive() && Math.abs(foundAngle) > 2) {
                foundAngle = foundTarget.estimateAngleToObject(AngleUnit.DEGREES);
                telemetry.addLine("Moving to SKY stone");
                telemetry.addData("Angle", foundAngle);
                telemetry.update();
               // sleep(500);


                if (foundAngle >= 0) {
                    robot.RobotMoveY(new Waypoint(0, -2, 0), 0.3);
                } else {
                    robot.RobotMoveY(new Waypoint(0, 2, 0), 0.3);
                }
                robot.Stop();
                TimeElapsedPause(100);


                 targets = _vision.visionFeedback(telemetry);

                if(targets == null || targets.size() == 0){
                    break;
                }

                //refind the target
                for (int count = 0; count < targets.size(); count++) {

                    if (targets.get(count).getLabel() == "Skystone") {
                        foundTarget = targets.get(count);
                        break;
                    }
                }

            }
            //sleep(2000);

            telemetry.addLine("LOCKED ONTO Stone");
            telemetry.addData("Angle", foundAngle);
            telemetry.addData("Stone", foundTarget.getLabel());
            telemetry.update();

           // sleep(2000);

            robot.RobotMoveX(new Waypoint(35, 0, 0 ), 0.2);
            robot.Stop();
            robot.SkyHookOn();
            sleep(1500);
            robot.RobotMoveX(new Waypoint(-40, 0, 0 ), 0.2);
            robot.Stop();
            sleep(2000);

        }else{

            robot.Stop();
            telemetry.addLine("No stones found - parking");
            //look at the localiser and work out how far back we need to go.


        }

        _vision.cleanUp();

    }

    public void TimeElapsedPause(double ms){

        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < ms){
            idle();
        }

    }


}
