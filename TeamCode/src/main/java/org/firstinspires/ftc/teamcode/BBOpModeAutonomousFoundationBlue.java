package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-Foundation-Blue", group="BB")
public class BBOpModeAutonomousFoundationBlue extends LinearOpMode
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

        //TODO: work out how far forward we need to move

        //TODO: put down the hooks into the foundation

        //TODO: work out how far backwards we need to move

        //TODO: we need to move out onto the line.


    }


}
