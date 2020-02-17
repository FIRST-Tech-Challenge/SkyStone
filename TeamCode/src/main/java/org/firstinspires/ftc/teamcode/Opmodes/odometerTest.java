package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

@Autonomous(name="Odometer Test", group="Testing")
public class odometerTest extends LinearOpMode {

    // Declare OpMode Members
    private RobotHardware robotHardware = new RobotHardware(hardwareMap);
    private OdometerIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private Timer timer;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);

        while(opModeIsActive()){
            odometer.update();

            telemetry.addData("X", odometer.x);
            telemetry.addData("Y", odometer.y);
            telemetry.addData("Heading", odometer.heading);
            telemetry.update();

        }

    }

    private void initialize(){
        robotHardware.hardwareMap();

        odometer = new OdometerIMU2W(robotHardware);
        drivetrain = new MecanumDrive(robotHardware);
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer);
        drivetrain.initialize();
        odometer.initialize();

    }
}
