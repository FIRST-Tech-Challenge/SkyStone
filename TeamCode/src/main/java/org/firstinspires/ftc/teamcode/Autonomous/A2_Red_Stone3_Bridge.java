package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;

@Disabled
@TeleOp (name = "A2_Red_Stone3_Bridge")

public class A2_Red_Stone3_Bridge extends LinearOpMode {

    OrientationTools orientationTools;
    HardwareChassisGyro robotGyro;
    HardwareChassis robot;
    GeneralTools generalTools;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;
    ControlledDrive controlledDrive;
    ColorTools colorTools;
    OmniWheel omniWheel;

    double startPos;
    double extenderEncoderValue = 2.8;
    double extenderFoundationValue = 4;
    double liftEncoderValue = 1.5;
    double liftStartOffset = 0.75;
    double liftFoundationValue = 1.6;
    double ap_forwardGrabStone = 70;

    @Override
    public void runOpMode() {

        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, telemetry);
        controlledExtender = new ControlledExtender(robot, telemetry);
        orientationTools = new OrientationTools(robot, hardwareMap, this);
        robotGyro = new HardwareChassisGyro(hardwareMap);

        startPos = orientationTools.getDegree360(robotGyro.imu);

        waitForStart();

        if (opModeIsActive()) {
            generalTools.openClamp();
            generalTools.releaseFoundation();
            controlledLift.start(liftFoundationValue,0.6);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }
        // you lifted the lift up

        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.6);
            orientationTools.driveSidewardEncoder(this, 0, -60, 0.4, omniWheel, startPos, robotGyro.imu, 175, 125);
            while (!controlledExtender.endReached() && opModeIsActive()) { }
            controlledExtender.stop();
        }

        // you have now extended the arm and driven sidewards to A1

        if (opModeIsActive()) {
            controlledDrive.start(ap_forwardGrabStone, 0, 0.4);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }

        // you are now standing in front of one stone

        if (opModeIsActive()) {
            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.6);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            //lowers the lift

            generalTools.stopForMilliSeconds(500);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }

        // you should have grabbed the second stone from the wall now

        if (opModeIsActive()) {
            controlledDrive.start(-20, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, 170, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        // you should now have crossed the bridge
        // B4

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have now dropped the stone

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -15, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
            generalTools.closeClamp();
        }

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -90, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
            generalTools.openClamp();
        }

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -30, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        // you have now crossed the bridge again
        // B1

        if (opModeIsActive()) {
            controlledDrive.start(20, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you should now be in front of the third stone from the wall

        if (opModeIsActive()) {
            generalTools.closeClamp();
        }

        // you should have grabbed the 2nd stone now

        if (opModeIsActive()) {
            controlledDrive.start(-20, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, 150, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        // you have now delivered the 2nd stone

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have dropped the 2nd stone now

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -15, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
            generalTools.closeClamp();
        }

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -90, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
            generalTools.openClamp();
        }

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -10, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        // you have now crossed the bridge again
        // B2


        if (opModeIsActive()) {
            controlledDrive.start(20, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you should now be in front of the fourth stone from the wall

        if (opModeIsActive()) {
            generalTools.closeClamp();
        }

        // you should have grabbed the 2nd stone now

        if (opModeIsActive()) {
            controlledDrive.start(-20, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }
        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, 130, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have delivered the 4th stone

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -40, 0.6, omniWheel, startPos, robotGyro.imu, 175, 125);
        }

        // you parked below the bidge
    }
}
