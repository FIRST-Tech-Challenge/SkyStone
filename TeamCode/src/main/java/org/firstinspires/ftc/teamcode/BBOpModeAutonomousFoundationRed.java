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
    private BBVision _vision = new BBVision();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);
        hooks.init(hardwareMap);
        _vision.setUp(telemetry,hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //TODO: work out how far forward we need to move
        robot.moveForward(100, 0.60);
        sleep(1000);

        //TODO: put down the hooks into the foundation
        hooks.Latched();
        sleep(2000);

        //TODO: work out how far backwards we need to move
        robot.moveBackwards(100, 0.60);
        sleep(500);

        //TODO: we need to move out onto the line.
        robot.turnLeft(90, 0.5);
        sleep(1000);
        //robot.moveForward(100, 0.6);


    }


}
