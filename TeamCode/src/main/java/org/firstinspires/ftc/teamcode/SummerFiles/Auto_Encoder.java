package org.firstinspires.ftc.teamcode.SummerFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in
 * either the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name = "Encoder (beta)", group = "Autonomous")
public class Auto_Encoder extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double TPI_F = 1120 / (Math.PI * 4);
    private double TPI_B = TPI_F * 0.5;

    private Trobot trobot;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        telemetry.addData("Status", "Waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        trobot.getDrivetrain().drive(0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafe(-1, 0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().drive(-0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        trobot.getDrivetrain().strafe(1, 0.5);
        sleep(3000);

        trobot.getDrivetrain().stop();
        sleep(1000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //}
    }
}
