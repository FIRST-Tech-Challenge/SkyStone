package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CustomCV.RedPipeline;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="Full Red Auto", group="Auto")
public class redAuto extends LinearOpMode {

    // Declare OpMode Members
    private RobotHardware hardware = new RobotHardware();
    private Timer timer;
    private OdometerKIMU2W odometer;
    private MecanumDrive drivetrain;
    private Movement movement;
    private AutoClaws autoClaws;
    private RedPipeline pipeline;
    private OpenCvCamera phoneCam;

    private int skyPosition;
    private ActionHandlerClaws handler;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.start();
        odometer.startTracking(0, 0, 0);
        telemetry.addData("status","running");
        telemetry.update();

        scanSkystone();

        // Initialization of Paths
        ArrayList<RobotPoint> pickupBlockOne = new ArrayList<>();
        pickupBlockOne.add(new RobotPoint(237, 74, 90, 1.0)); // at foundation
        pickupBlockOne.add(new RobotPoint(175, 69, 90, 1.0)); // at foundation
        pickupBlockOne.add(new RobotPoint(156, 63, 90, 1.0));
        pickupBlockOne.add(new RobotPoint(115, 63, 90, 1.0));
        pickupBlockOne.add(new RobotPoint(86, 70, 90, 1.0)); // at blo

        ArrayList<RobotPoint> pickupBlockTwo = new ArrayList<>();
        pickupBlockTwo.add(new RobotPoint(237, 74, 90, 1.0)); // at foundation
        pickupBlockTwo.add(new RobotPoint(175, 69, 90, 1.0)); // at foundation
        pickupBlockTwo.add(new RobotPoint(156, 63, 90, 1.0));
        pickupBlockTwo.add(new RobotPoint(115, 63, 90, 1.0));
        pickupBlockTwo.add(new RobotPoint(86, 68, 90, 1.0));
        pickupBlockTwo.add(new RobotPoint(66, 70, 90, 1.0)); // at block

        ArrayList<RobotPoint> pickupBlockThree = new ArrayList<>();
        pickupBlockThree.add(new RobotPoint(237, 74, 90, 1.0)); // at foundation
        pickupBlockThree.add(new RobotPoint(175, 69, 90, 1.0)); // at foundation
        pickupBlockThree.add(new RobotPoint(156, 63, 90, 1.0));
        pickupBlockThree.add(new RobotPoint(115, 63, 90, 1.0));
        pickupBlockThree.add(new RobotPoint(86, 68, 90, 1.0));
        pickupBlockThree.add(new RobotPoint(66, 70, 90, 1.0)); // at block
        pickupBlockThree.add(new RobotPoint(46, 70, 90, 1.0)); // at block

        ArrayList<RobotPoint> pickupBlockFour = new ArrayList<>();
        pickupBlockFour.add(new RobotPoint(237, 74, 90, 1.0)); // at foundation
        pickupBlockFour.add(new RobotPoint(175, 69, 90, 1.0)); // at foundation
        pickupBlockFour.add(new RobotPoint(156, 63, 90, 1.0));
        pickupBlockFour.add(new RobotPoint(115, 63, 90, 1.0));
        pickupBlockFour.add(new RobotPoint(86, 68, 90, 1.0));
        pickupBlockFour.add(new RobotPoint(66, 68, 90, 1.0)); // at block
        pickupBlockFour.add(new RobotPoint(46, 70, 90, 1.0)); // at block
        pickupBlockFour.add(new RobotPoint(26, 72, 90, 1.0)); // at block

        ArrayList<RobotPoint> dropoffOne = new ArrayList<>();
        dropoffOne.add(new RobotPoint(46,74.5,90,1.0)); // at block
        dropoffOne.add(new RobotPoint(60,70,90,1.0));
        dropoffOne.add(new RobotPoint(86, 65, 90, 1.0));
        dropoffOne.add(new RobotPoint(115, 61, 90, 1.0));
        dropoffOne.add(new RobotPoint(156, 65, 90, 1.0));
        dropoffOne.add(new RobotPoint(175, 73, 90, 1.0)); // at foundation
        dropoffOne.add(new RobotPoint(236, 79, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> dropoffTwo = new ArrayList<>();
        dropoffTwo.add(new RobotPoint(25,74.5,90,1.0)); // at block
        dropoffTwo.add(new RobotPoint(45,72,90,1.0)); // at block
        dropoffTwo.add(new RobotPoint(60,70,90,1.0));
        dropoffTwo.add(new RobotPoint(86, 65, 90, 1.0));
        dropoffTwo.add(new RobotPoint(115, 63, 90, 1.0));
        dropoffTwo.add(new RobotPoint(156, 67, 90, 1.0));
        dropoffTwo.add(new RobotPoint(175, 73, 90, 1.0)); // at foundation
        dropoffTwo.add(new RobotPoint(234, 79, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> dropoffThree = new ArrayList<>();
        dropoffThree.add(new RobotPoint(5,74.5,90,1.0)); // at block
        dropoffThree.add(new RobotPoint(30,68,90,1.0)); // at block
        dropoffThree.add(new RobotPoint(61, 65, 90, 1.0));
        dropoffThree.add(new RobotPoint(115, 63, 90, 1.0));
        dropoffThree.add(new RobotPoint(156, 67, 90, 1.0));
        dropoffThree.add(new RobotPoint(165, 73, 90, 1.0)); // at foundation
        dropoffThree.add(new RobotPoint(220, 79, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> dropoffFour = new ArrayList<>();
        dropoffFour.add(new RobotPoint(-18,74.5,90,1.0)); // at block
        dropoffFour.add(new RobotPoint(30,68,90,1.0)); // at block
        dropoffFour.add(new RobotPoint(61, 65, 90, 1.0));
        dropoffFour.add(new RobotPoint(115, 63, 90, 1.0));
        dropoffFour.add(new RobotPoint(156, 67, 90, 1.0));
        dropoffFour.add(new RobotPoint(165, 73, 90, 1.0)); // at foundation
        dropoffFour.add(new RobotPoint(220, 79, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> dropoffFive = new ArrayList<>();
        dropoffFive.add(new RobotPoint(-38,74.5,90,1.0)); // at block
        dropoffFive.add(new RobotPoint(46,68,90,1.0)); // at block
        dropoffFive.add(new RobotPoint(61, 65, 90, 1.0));
        dropoffFive.add(new RobotPoint(115, 63, 90, 1.0));
        dropoffFive.add(new RobotPoint(156, 67, 90, 1.0));
        dropoffFive.add(new RobotPoint(155, 73, 90, 1.0)); // at foundation
        dropoffFive.add(new RobotPoint(220, 77, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> dropoffSix = new ArrayList<>();
        dropoffSix.add(new RobotPoint(-56,74.5,90,1.0)); // at block
        dropoffSix.add(new RobotPoint(46,68,90,1.0)); // at block
        dropoffSix.add(new RobotPoint(61, 65, 90, 1.0));
        dropoffSix.add(new RobotPoint(115, 61, 90, 1.0));
        dropoffSix.add(new RobotPoint(156, 65, 90, 1.0));
        dropoffSix.add(new RobotPoint(155, 73, 90, 1.0)); // at foundation
        dropoffSix.add(new RobotPoint(220, 77, 90, 1.0)); // at foundation

        ArrayList<RobotPoint> park = new ArrayList<>();
        park.add(new RobotPoint(230,40,90,0.4)); // at block
        park.add(new RobotPoint(200,70,90,0.4)); // at block

        // Actual running of routine
        if (skyPosition == 0){//closest wall
            autoClaws.firstPrime();
            movement.moveToPointPD(new RobotPoint(-56, 73, 90, 0.1), 100, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffSix,15);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockThree, 16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(5, 76, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffThree,16);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockOne,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(46, 77, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffOne,15);
            autoClaws.depositBlock();
            movement.followPath(pickupBlockTwo,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(28, 79, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffTwo,15);
            autoClaws.depositBlock();


        } if(skyPosition == 1 ){//middle
            autoClaws.firstPrime();
            movement.moveToPointPD(new RobotPoint(-38, 73, 90, 0.1), 100, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffFive,15);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockTwo,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(28, 76, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffTwo,15);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockOne,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(48, 77, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffOne,15);
            autoClaws.depositBlock();
            movement.followPath(pickupBlockThree,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(8, 79, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffThree,15);
            autoClaws.depositBlock();

        }if(skyPosition == 2){//furthest from wall
            autoClaws.firstPrime();
            movement.moveToPointPD(new RobotPoint(-18,73, 90, 0.1), 100, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffFour,15);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockOne,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(48, 76, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffTwo,15);
            autoClaws.depositBlockThrow();
            movement.followPath(pickupBlockTwo,16);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(28, 77, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffTwo,15);
            autoClaws.depositBlock();
            movement.followPath(pickupBlockThree,15);
            autoClaws.prime();
            movement.moveToPointPD(new RobotPoint(8, 79, 90, 0.1), 50, 1.5);
            autoClaws.grabBlock();
            autoClaws.storeBlock();
            movement.followPath(dropoffThree,15);
            autoClaws.depositBlock();

        }

        movement.pointInDirection(180, 15);
        movement.deadReckon(0, 0.9, 0, 475);
        clampFoundation();
        movement.deadReckon(0.2, -0.7, -0.6, 1550);
        movement.deadReckon(0.3, -0.4, -0.8, 1500);
        releaseFoundation();
        movement.followPath(park,15);
        movement.moveToPointPD2(new RobotPoint(97, 67, -90, 0.1), 50, 1.5);
        //movement.followPath(pickupBlockOne, 30);
        //movement.moveToPointPD(new RobotPoint(0, 0, 90, 0.1), 100, 1.5);

    }

    private void scanSkystone(){
        skyPosition = pipeline.getRedPosition();

        if(skyPosition == 404) {
            scanSkystone();
        }
    }

    private void clampFoundation(){
        hardware.foundationClampLeft.setPosition(.20);
        hardware.foundationClampRight.setPosition(.80);
        timer.waitMillis(100);
    }

    private void releaseFoundation(){
        hardware.foundationClampLeft.setPosition(0.745);
        hardware.foundationClampRight.setPosition(0.26);
        timer.waitMillis(100);
    }

    private void initialize(){
        hardware.hardwareMap(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        pipeline = new RedPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        drivetrain = new MecanumDrive(this, hardware);
        odometer = new OdometerKIMU2W(this, hardware);
        timer = new Timer(this, odometer);
        autoClaws = new AutoClaws(hardware, "RED", timer);
        handler = new ActionHandlerClaws(autoClaws);
        movement = new Movement(this, drivetrain, odometer, timer);
        movement.setActionHandler(handler);
        movement.useActionHandler = true;

        hardware.foundationClampLeft.setPosition(0.745);
        hardware.foundationClampRight.setPosition(0.26);

        autoClaws.initialize();
        drivetrain.initialize();
        odometer.initialize();

        telemetry.addData("status","initialized");
        telemetry.update();
    }
}

/*
 RobotPoint point1 = new RobotPoint( -38, 74, 90, 0.9);
        movement.moveToPointConstants(point1, 0.8, 0.11, 20, 3);
        autoClaws.grabBlock();

        ArrayList<RobotPoint> delivery = new ArrayList<>();
        RobotPoint point2 = new RobotPoint(0, 0, 90, 0.9);
        RobotPoint point3 = new RobotPoint(97, 57, 90, 0.9);
        point3.setHookActions(0.482, 0.985);
        RobotPoint point4 = new RobotPoint(215, 80, 90, 0.9);
        point4.setHookActions(0.482, 0.985);
        delivery.add(point2);
        delivery.add(point3);
        delivery.add(point4);
        movement.followPath(delivery, 40);
        movement.moveToPointConstants(point4, 0.8, 0.3, 15, 4);
        autoClaws.depositBlock();
*/