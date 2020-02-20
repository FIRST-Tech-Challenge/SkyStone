package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandler;
import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="Red Auto", group="Auto")
public class redSideAuto extends LinearOpMode {

    // Declare OpMode Members
    private Timer timer;
    private OdometerKIMU2W odometer;
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

        RobotPoint point1 = new RobotPoint( -38, 74, 90, 0.9, 0);
        movement.movetoPointConstants(point1, 0.8, 0.11, 20, 3);
        autoClaws.grabBlock();

        ArrayList<RobotPoint> delivery = new ArrayList<>();
        RobotPoint point2 = new RobotPoint(0, 0, 90, 0.9, 10);
        RobotPoint point3 = new RobotPoint(97, 57, 90, 0.9, 10);
        point3.setHookActions(0.482, 0.985);
        RobotPoint point4 = new RobotPoint(215, 80, 90, 0.9, 10);
        point4.setHookActions(0.482, 0.985);
        delivery.add(point2);
        delivery.add(point3);
        delivery.add(point4);
        movement.followPath(delivery);
        movement.movetoPointConstants(point4, 0.8, 0.3, 15, 4);
        autoClaws.depositBlock();
    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive();
        odometer = new OdometerKIMU2W();
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
