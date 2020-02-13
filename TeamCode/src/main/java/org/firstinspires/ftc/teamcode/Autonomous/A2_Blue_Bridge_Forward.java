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

@Autonomous (name =  "A2_Blue_Bridge_Forward")

public class A2_Blue_Bridge_Forward extends LinearOpMode {

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

        if (opModeIsActive()){
            generalTools.releaseFoundation();
        }

        if (opModeIsActive()) {
            controlledLift.start(liftFoundationValue,0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have now uplifted the lift

        if (opModeIsActive()){
            controlledExtender.start(extenderEncoderValue,0.4);
            while (!controlledExtender.endReached()) {}
            controlledExtender.stop();
            controlledLift.start(-(liftFoundationValue + liftStartOffset),0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have now lowered the lift and pulled out the arm

        if (opModeIsActive()) {
            controlledDrive.start(52, 0, 0.6);
            while (!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you are now on B2

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -generalTools.bcap_underBridge, -0.6, omniWheel, startPos, robotGyro.imu, 175, 150);
        }

        // you are now below the bridge

    }



}
