package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandler;
import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="Action System Test", group="Testing")
public class actionsTest extends LinearOpMode {

    // Declare OpMode Members
    private Timer timer;
    private OdometerIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private AutoClaws autoClaws;

    private ActionHandlerClaws handler;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        /*
        ArrayList<RobotPoint> testPath = new ArrayList<>();

        RobotPoint point1 = new RobotPoint(0, 0, 0, 0.7, 10);
        point1.addActions(0.7, 0.575, 0);
        RobotPoint point2 = new RobotPoint(38, 74, 0, 0.7, 10);
        point2.addActions(0.7, 0.575, 0);
        RobotPoint point3 = new RobotPoint(55, 55, 0, 0.7, 0);
        testPath.add(point1);
        testPath.add(point2);
        testPath.add(point3);

        movement.followPath(testPath);
        */

        RobotPoint point = new RobotPoint(50, 50, 0, 0);
        point.setHookActions(0.99, 0.985);
        movement.moveToPointConstants(point, 0.4, 0.3, 3, 2);

        timer.waitMillis(500);
        autoClaws.grabBlock();

    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive();
        odometer = new OdometerIMU2W();
        timer = new Timer(this, odometer);
        autoClaws = new AutoClaws("RED", timer);
        movement = new Movement(this, drivetrain, odometer);
        movement.setActionHandler(handler);
        movement.useActionHandler = true;

        autoClaws.initialize();
        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
