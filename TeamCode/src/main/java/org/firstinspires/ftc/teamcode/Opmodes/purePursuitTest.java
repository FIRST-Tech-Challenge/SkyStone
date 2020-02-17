package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import java.util.ArrayList;

@Autonomous(name="Pure Pursuit Test", group="Testing")
public class purePursuitTest extends LinearOpMode {

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

        ArrayList<RobotPoint> testPath = new ArrayList<>();
        testPath.add(new RobotPoint(0, 50, 0));
        testPath.add(new RobotPoint(50, 50, 0));

        movement.followPath(6, testPath);

        drivetrain.freeze();

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
