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

@Autonomous(name="Intake Auto", group="Auto")
public class intakeBlockAuto extends LinearOpMode {

    // Declare OpMode Members
    private OdometerIMU2W odometer;
    private Timer timer;
    private MecanumDrive drivetrain;
    private Movement movement;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        ArrayList<RobotPoint> intakePath = new ArrayList<>();
        intakePath.add(new RobotPoint(0, 0, 0, 0.5, 7));
        intakePath.add(new RobotPoint(49, 40, 39, 0.5, 5));
        intakePath.add(new RobotPoint(24, 91, 44, 0.5, 5));
        intakePath.add(new RobotPoint(24, 91, 38, 0.5, 5));//This point is the final point, threshold
        RobotPoint intakePathGoal = new RobotPoint(18, 101, 23, 0.5,0);
        intakePath.add(intakePathGoal); //Extension of the path to keep the robot moving, the eventual goal of the movement

        ArrayList<RobotPoint> deliverPath = new ArrayList<>();
        deliverPath.add(new RobotPoint(15, 27, 60, 0.7, 10));
        deliverPath.add(new RobotPoint(30, 10, 80, 0.5, 5));//This point is the final point, threshold
        RobotPoint deliverPathGoal = new RobotPoint(59, 3, 90, 0.5,0);
        deliverPath.add(deliverPathGoal); //Extension of the path to keep the robot moving, the eventual goal of the movement

        movement.followPath(intakePath);
        movement.movetoPointConstants(intakePathGoal, 0.4, 0.2, 3, 2);

        timer.waitMillis(50);

        movement.followPath(deliverPath);
        movement.movetoPointConstants(deliverPathGoal, 0.4, 0.3, 3, 2);

    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive();
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer);
        movement.useActionHandlers = false;
        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
