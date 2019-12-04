package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    private DcMotor drive_FL;
    private DcMotor drive_FR;
    private DcMotor drive_RL;
    private DcMotor drive_RR;

    private DcMotor intake_L;
    private DcMotor intake_R;

    private Servo latch_L;
    private Servo latch_R;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive_FL = hardwareMap.get(DcMotor.class, "front left");
        drive_FR = hardwareMap.get(DcMotor.class, "front right");
        drive_RL = hardwareMap.get(DcMotor.class, "rear left");
        drive_RR = hardwareMap.get(DcMotor.class, "rear right");

        intake_L  = hardwareMap.get(DcMotor.class, "left intake");
        intake_R = hardwareMap.get(DcMotor.class, "right intake");

        latch_L = hardwareMap.get(Servo.class, "left servo");
        latch_R = hardwareMap.get(Servo.class, "right servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        drive_FL.setDirection(DcMotor.Direction.REVERSE);
        drive_FR.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.FORWARD);
        drive_RR.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        drive_FL.setDirection(DcMotor.Direction.FORWARD);
        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RL.setDirection(DcMotor.Direction.REVERSE);
        drive_RR.setDirection(DcMotor.Direction.FORWARD);

        drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive_FL.setTargetPosition((int)(10 * TPI_F));
        drive_FR.setTargetPosition((int)(10 * TPI_B));
        drive_RL.setTargetPosition((int)(10 * TPI_F));
        drive_RR.setTargetPosition((int)(10 * TPI_B));

        drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive_FL.setPower(1);
        drive_FR.setPower(0.5);
        drive_RL.setPower(1);
        drive_RR.setPower(0.5);

        while((drive_FL.isBusy() || drive_FR.isBusy() || drive_RL.isBusy() || drive_RR.isBusy()) && opModeIsActive()){
            idle();
        }

        drive_FL.setPower(0);
        drive_FR.setPower(0);
        drive_RL.setPower(0);
        drive_RR.setPower(0);

        sleep(1000);

        latch_L.setPosition(0.35);
        latch_R.setPosition(0.75);

        sleep(1000);

        drive_FL.setDirection(DcMotor.Direction.REVERSE);
        drive_FR.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.FORWARD);
        drive_RR.setDirection(DcMotor.Direction.REVERSE);

        drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive_FL.setTargetPosition((int)(24 * TPI_F));
        drive_FR.setTargetPosition((int)(24 * TPI_B));
        drive_RL.setTargetPosition((int)(24 * TPI_F));
        drive_RR.setTargetPosition((int)(24 * TPI_B));

        drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive_FL.setPower(1);
        drive_FR.setPower(0.5);
        drive_RL.setPower(1);
        drive_RR.setPower(0.5);

        while(drive_FL.getCurrentPosition() > 0 && opModeIsActive()){
            idle();
        }

        drive_FL.setPower(0);
        drive_FR.setPower(0);
        drive_RL.setPower(0);
        drive_RR.setPower(0);

        sleep(10000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //}
    }
}
