package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSystems.ActionHandlerClaws;
import org.firstinspires.ftc.teamcode.HardwareSystems.AutoClaws;
import org.firstinspires.ftc.teamcode.Movement.Localization.OdometerKIMU2W;
import org.firstinspires.ftc.teamcode.Movement.MecanumDrive;
import org.firstinspires.ftc.teamcode.Movement.MotionPlanning.RobotPoint;
import org.firstinspires.ftc.teamcode.Movement.Movement;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.ArrayList;

@Autonomous(name="Blue Auto", group="Auto")
public class blueSideAuto extends LinearOpMode {

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

        int skyPosition = 0;

        //GRAB FIRST BLOCK
        if(skyPosition == 0) { //Closest to wall
            movement.moveToPointPD(new RobotPoint(56, 74.5, -90, 0,0), 100, 2);

            timer.waitMillis(3000);

        }else if(skyPosition == 1) { //Middle Stone
        }else if(skyPosition == 2) { //Furthest From Wall
        }

        ArrayList<RobotPoint> deliverPath = new ArrayList<>();
        deliverPath.add(new RobotPoint(55, 75.5, -90, 1, 50));
        deliverPath.add(new RobotPoint(-40, 40, -90, 0.8, 50));
        deliverPath.add(new RobotPoint(-105, 53, -90, 1, 50));
        deliverPath.add(new RobotPoint(-165, 63, -90, 1, 20));
        deliverPath.add(new RobotPoint(-175, 65, -90, 1, 0));

        ArrayList<RobotPoint> returnPath = new ArrayList<>();
        returnPath.add(new RobotPoint(-175, 65, -90, 1, 50));
        returnPath.add(new RobotPoint(-165, 63, -90, 1, 50));//added
        returnPath.add(new RobotPoint(-105, 53, -90, 1, 50));
        returnPath.add(new RobotPoint(-70, 45, -90, 1, 50));//added
        returnPath.add(new RobotPoint(-40, 40, -90, 0.8, 50));
        returnPath.add(new RobotPoint(55, 75.5, -90, 1, 0));

        movement.followPath(deliverPath);
        timer.waitMillis(3000);
        movement.followPath(returnPath);


        //GRAB SECOND BLOCK
        if(skyPosition == 0) { //Closest to wall
            //movement.moveToPointPD2(new RobotPoint(-5, 75, -90, 0, 0), 50,2);
        }else if(skyPosition == 1) { //Middle Stone
        }else if(skyPosition == 2) { //Furthest From Wall
        }

    }

    private void initialize(){
        RobotHardware.hardwareMap(hardwareMap);

        drivetrain = new MecanumDrive();
        odometer = new OdometerKIMU2W();
        timer = new Timer(this, odometer);
        autoClaws = new AutoClaws("BLUE", timer);
        handler = new ActionHandlerClaws(autoClaws);
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
