package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerIntake;
import org.firstinspires.ftc.teamcode.HardwareSystems.Extrusion;
import org.firstinspires.ftc.teamcode.HardwareSystems.Intake;
import org.firstinspires.ftc.teamcode.HardwareSystems.Outtake;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerIMU2W;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="Intake Auto", group="Auto")
public class intakeBlockAuto extends LinearOpMode {

    // Declare OpMode Members
    private OdometerKIMU2W odometer;
    private Timer timer;
    private MecanumDrive drivetrain;
    private Movement movement;

    private Intake intake;
    private Outtake outtake;
    private Extrusion extrusion;
    private ActionHandlerIntake handler;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        //Write paths here

        //Write paths here
        ArrayList<RobotPoint> intakePath = new ArrayList<>();
        intakePath.add(new RobotPoint(0, 0, 0, 0.5, 10));
        intakePath.add(new RobotPoint(32, 52, 21, 0.5, 8));
        intakePath.add(new RobotPoint(30, 76, 27, 0.5, 7));
        intakePath.add(new RobotPoint(16, 104, 32, 0.5, 7));//This point is the final point, threshold
        RobotPoint intakePathGoal = new RobotPoint(14, 130, 20, 0.5,0);

        intakePath.add(intakePathGoal); //Extension of the path to keep the robot moving, the eventual goal of the movement

        ArrayList<RobotPoint> deliverPath = new ArrayList<>();
        deliverPath.add(new RobotPoint(30, 95, 20, 0.7, 8));
        deliverPath.add(new RobotPoint(30, 76, 20, 0.7, 8));
        deliverPath.add(new RobotPoint(41, 50, 70, 0.7, 8));
        deliverPath.add(new RobotPoint(65, 47, 90, 0.7, 8));
        deliverPath.add(new RobotPoint(122, 47, 90, 0.7, 8));
        deliverPath.add(new RobotPoint(162, 45, 135, 0.7, 8));
        deliverPath.add(new RobotPoint(205, 53, 180, 0.7, 8));
        RobotPoint deliverPathGoal = new RobotPoint(205, 53, 180, 0.5,0);
        deliverPath.add(deliverPathGoal);

        deliverPath.add(deliverPathGoal); //Extension of the path to keep the robot moving, the eventual goal of the movement


        //Write autonomous program
        intake.setPower(0.8);

        movement.followPath(intakePath);
        telemetry.addData("status", "made it 1");
        telemetry.update();
        //movement.movetoPointConstants(intakePathGoal, 0.3, 0.2, 3, 4);

        intake.setPower(0);
        outtake.setGripperState("Clamped");
        telemetry.addData("status", "made it 2");
        telemetry.update();
        movement.followPath(deliverPath);
      //  movement.movetoPointConstants(deliverPathGoal, 0.3, 0.2, 3, 4);


    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive();
        odometer = new OdometerKIMU2W();
        timer = new Timer(this, odometer);
        movement = new Movement(this, drivetrain, odometer);
        intake = new Intake();
        outtake = new Outtake();
        extrusion = new Extrusion(this);
        handler = new ActionHandlerIntake(intake, outtake, extrusion);
        //movement.useActionHandler = true;
        movement.setActionHandler(handler);

        drivetrain.initialize();
        odometer.initialize();
        intake.initialize();
        outtake.initialize();
        extrusion.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();

    }
}


//DELIVERY PATH 2/21/2020

        /*
        deliverPath.add(new RobotPoint(14, 120, 50, 0.7, 8));

        deliverPath.add(new RobotPoint(10, 45, 88, 0.5, 10));
        deliverPath.add(new RobotPoint(69, 43, 90, 0.5, 10));
        deliverPath.add(new RobotPoint(196, 42, 150, 0.5, 10));//This point is the final point, threshold
        RobotPoint deliverPathGoal = new RobotPoint(198, 48, 180, 0.5,0);
        deliverPath.add(deliverPathGoal); //Extension of the path to keep the robot moving, the eventual goal of the movement
*/

                /*
        deliverPath.add(new RobotPoint(30, 76, 20, 0.7, 8));
        deliverPath.add(new RobotPoint(41, 50, 70, 0.7, 8));
        deliverPath.add(new RobotPoint(65, 47, 90, 0.7, 8));
        deliverPath.add(new RobotPoint(122, 47, 90, 0.7, 8));
        deliverPath.add(new RobotPoint(162, 45, 135, 0.7, 8));
        deliverPath.add(new RobotPoint(205, 53, 180, 0.7, 8));
        RobotPoint deliverPathGoal = new RobotPoint(205, 53, 180, 0.5,0);
        deliverPath.add(deliverPathGoal);

         */