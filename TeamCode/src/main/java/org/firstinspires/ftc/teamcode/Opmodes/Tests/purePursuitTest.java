package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.PathingAgent;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import java.util.ArrayList;

@Autonomous(name="Pure Pursuit Test", group="Testing")
@Disabled
public class purePursuitTest extends LinearOpMode {

    // Declare OpMode Members
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
        telemetry.addData("status","running");
        telemetry.update();

        ArrayList<RobotPoint> deliverPath = new ArrayList<>();
        deliverPath.add(new RobotPoint(55, 75.5, -90, 0.8));
        deliverPath.add(new RobotPoint(-40, 30, -90, 0.5));
        deliverPath.add(new RobotPoint(-165, 63, -90, 0.7));
        deliverPath.add(new RobotPoint(-175, 65, -90, 0.8));

        while(opModeIsActive()) {
            RobotPoint target = PathingAgent.getTargetPoint(odometer.x, odometer.y, 50, deliverPath);
            telemetry.addData("RobotX", odometer.x);
            telemetry.addData("RobotY", odometer.y);
            telemetry.addData("targetPointX", target.x);
            telemetry.addData("targetPointY", target.y);
            telemetry.update();
            odometer.update();
        }

    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        odometer = new OdometerIMU2W();
        drivetrain = new MecanumDrive(this);
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer, timer);

        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}
