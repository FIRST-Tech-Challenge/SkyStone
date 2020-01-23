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


@Autonomous(name = "C_Autonomous_Gyro")

public class ConceptGyroAutonomousRed extends LinearOpMode {

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
    double liftFoundationValue = 1.2;

    @Override
    public void runOpMode() {
        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, telemetry);
        controlledExtender = new ControlledExtender(robot, telemetry);
        orientationTools = new OrientationTools(robot);
        robotGyro = new HardwareChassisGyro(hardwareMap);


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
            controlledExtender.start(extenderEncoderValue, 0.4);

            controlledDrive.start(generalTools.ap_forwardGrabStone, 0, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();

            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.2); //lowers the lift
            generalTools.stopForMilliSeconds(500);

            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(1000);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }

        // hey... you should have grabbed a stone now...

        if (opModeIsActive()) {
            controlledDrive.start(-16, 0, 0.4);
            while (!controlledDrive.endReached()) {}
            controlledDrive.stop();
        }

        // you have driven back a few cm

        if (opModeIsActive()) {
            orientationTools.driveSidewardTime(2900, 0.5, 500, robotGyro.imu, omniWheel, this);
        }

        if (opModeIsActive()) {
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now on the other side of the bridge

        if (opModeIsActive()) {
            controlledLift.start(liftFoundationValue, 0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have now uplifted the arm

        if (opModeIsActive()) {
            controlledExtender.start(extenderFoundationValue, 0.2);
            while (!controlledExtender.endReached()) {}
            controlledExtender.stop();
        }


        if (opModeIsActive()) {
            while (!colorTools.isRed(robot.color_front) && opModeIsActive()) {
                omniWheel.setMotors(0.4, 0, 0);
            }
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now standing right in front of the foundation

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have now released the stone on the foundation

        if (opModeIsActive()) {
            generalTools.grabFoundation();
            generalTools.stopForMilliSeconds(1000);
        }

        //you have now grabbed the foundation

        if (opModeIsActive()) {
            generalTools.backTillButtons(robot);
        }

        // you have now dragged the foundation into the corner (in best case)

        if (opModeIsActive()){
            generalTools.releaseFoundation();
        }

        // you have now released the foundation

        if (opModeIsActive()) {
            controlledDrive.start(0, -80, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you are now next to the foundation

        if (opModeIsActive()) {
            generalTools.backTillButtons(robot);
        }


        if (opModeIsActive()) {
            controlledExtender.start(-extenderFoundationValue, 0.2);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }

        // you have now put the arm back in

        if (opModeIsActive()) {
            controlledLift.start(-liftFoundationValue, 0.2); //distance 0.5
            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();

        }

        // you have now lifted the lift down

        if (opModeIsActive()) {
            controlledDrive.start(50, 0, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you are now on B4

        if (opModeIsActive()) {
            orientationTools.driveSidewardTime(800, -0.4, 500, robotGyro.imu, omniWheel, this);
        }

        //you are now parked under the bridge at B3/B4

    }
}