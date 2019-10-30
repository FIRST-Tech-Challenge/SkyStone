package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-Foundation-Blue", group="BB")
public class BBOpModeAutonomousFoundationBlue extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBHooks hooks = new BBHooks();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);
        hooks.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        hooks.UnLatched();
        sleep(1000);
        //work out how far forward we need to move
        robot.moveForward(100, 0.40);
        sleep(1000);

        //put down the hooks into the foundation
        hooks.Latched();
        sleep(2000);

        //work out how far backwards we need to move
        robot.moveBackwards(100, 0.40);
        sleep(500);

        hooks.UnLatched();
        sleep(2000);

        //we need to move out onto the line.
        robot.turnRight(90, 0.4);
        sleep(1000);
        robot.moveForward(100, 0.4);


    }


}
