package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;
import org.firstinspires.ftc.teamcode.Library.OrientationTools;


@Autonomous (name = "A2_Red_Stone_Bridge_Forward")

public class A2_Red_Stone_Bridge_Forward extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;
    OrientationTools orientationTools;
    HardwareChassisGyro robotGyro;

    double extenderEncoderValue = 3.5;
    double extenderFoundationValue = 4;
    double liftEncoderValue = 1.5;
    double liftStartOffset = 0.75;
    double liftFoundationValue = 1.6;
    double startPos;

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
        }

        if (opModeIsActive()) {
            generalTools.releaseFoundation();
        }

        if (opModeIsActive()) {
            controlledLift.start(liftEncoderValue, 0.4);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.6);

            controlledDrive.start(generalTools.ap_forwardGrabStone, 0, 0.4);

            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();

            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.2); //lowers the lift

            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(1000);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }

        // hey... you should have grabbed a stone now...

        if (opModeIsActive()) {
            controlledDrive.start(-22, 0, 0.4); //forward -20
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, generalTools.bcap_passBridge, 0.4, omniWheel, startPos, robotGyro.imu, 175, 150); //sideways: 220 --> middle
        }

        if (opModeIsActive()) {
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now on the other side of the bridge

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have now released the stone!

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -30, -0.4, omniWheel, startPos, robotGyro.imu, 175, 150);
        }

        // you are now next to the stone

        if (opModeIsActive()) {
            generalTools.closeClamp();
        }

        generalTools.stopForMilliSeconds(500);

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -50, -0.4, omniWheel, startPos, robotGyro.imu, 175, 150);
        }

        // you are now below the bridge aye

    }
}