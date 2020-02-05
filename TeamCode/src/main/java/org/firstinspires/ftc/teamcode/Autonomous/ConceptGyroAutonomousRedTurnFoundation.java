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

@Disabled
@Autonomous(name = "C_Turn Foundation_Autonomous_Gyro_Red")

public class ConceptGyroAutonomousRedTurnFoundation extends LinearOpMode {

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
    double liftFoundationValue = 1.4;
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
            generalTools.releaseFoundation();
            controlledLift.start(liftFoundationValue,0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you lifted the lift up

        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.6);
            //extend the arm 3.5
            controlledDrive.start(generalTools.ap_forwardGrabStone, 0, 0.5);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            //drive forward 65

            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();

            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.6);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            //lowers the lift

            generalTools.stopForMilliSeconds(500);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }

        // hey... you should have grabbed a stone now...


        if (opModeIsActive()) {
            controlledDrive.start(-20, 0, 0.4); //20cm
            while (!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, 215, 0.4, omniWheel, startPos, robotGyro.imu, 175);
        }

        if (opModeIsActive()) {
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now on the other side of the bridge

        if (opModeIsActive()) {
            controlledLift.start(liftFoundationValue, 0.4);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have now uplifted the arm

        if (opModeIsActive()) {
            controlledExtender.start(extenderFoundationValue + extenderEncoderValue*1/4, 0.6);
            while (!controlledExtender.endReached()) {}
            controlledExtender.stop();
        }

        if (opModeIsActive()) {
            controlledDrive.start(23, 0, 0.2);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you are now standing right in front of the foundation

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have now released the stone on the foundation

        if (opModeIsActive()) {
            generalTools.grabFoundation();
            generalTools.stopForMilliSeconds(250);
        }

        //you have now grabbed the foundation

        if (opModeIsActive()){
            controlledDrive.start(-50, 0, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }

        if (opModeIsActive()) {
            orientationTools.turnToDegrees(90, 200, omniWheel, robotGyro.imu);
        }

        // the foundation should now be turned into the corner

        if (opModeIsActive()) {
            controlledExtender.start(-extenderFoundationValue, 0.6);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }

        // you have now put the arm back in

        if (opModeIsActive()) {
            controlledLift.start(-liftFoundationValue, 0.6); //distance 0.5
            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();

        }

        if (opModeIsActive()) {
            generalTools.closeClamp();
        }

        // you have now lifted the lift down


        if (opModeIsActive()) {
            // drive sidewards to B5
        }


        if (opModeIsActive()) {
            controlledDrive.start(-100, 0, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }
    }
}