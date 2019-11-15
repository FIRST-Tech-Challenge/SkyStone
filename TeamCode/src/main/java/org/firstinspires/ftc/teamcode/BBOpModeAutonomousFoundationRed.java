package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Auto-Foundation-Red", group="BB")
public class BBOpModeAutonomousFoundationRed extends LinearOpMode
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

        //hooks.UnLatched();
        sleep(1000);
        //work out how far forward we need to move
        //robot.moveForward(150, 0.55);
        sleep(1000);

        //put down the hooks into the foundation
        hooks.Latched();
        sleep(2000);

        //work out how far backwards we need to move
        //robot.twoPowerBackwards(135, 0.8, 0.4);
        robot.moveBackwards(130, 0.60);
        sleep(500);

        hooks.UnLatched();
        sleep(2000);


        //we need to move out onto the line.
        robot.turnLeft(95, 0.70);
        sleep(1000);
        //robot.moveForward(170, 0.60);
        //robot.strafe(0.7);

        sleep(2000);
        //robot.moveForward(0, 0);



    }


}
