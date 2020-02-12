package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;


@Autonomous (name = "A2_Red_Stone_Bridge_Forward")

public class A2_Red_Stone_Bridge_Forward extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;

    double extenderEncoderValue = 3.5;
    double extenderFoundationValue = 4;
    double liftEncoderValue = 1.5;
    double liftStartOffset = 0.75;
    double liftFoundationValue = generalTools.liftFoundationValue;

    @Override
    public void runOpMode() {
        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, telemetry);
        controlledExtender = new ControlledExtender(robot, telemetry);

        generalTools.openClamp();
        generalTools.releaseFoundation();
        controlledLift.start(liftEncoderValue, 0.2);

        waitForStart();

        controlledLift.stop();

        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.5);

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
            //controlledDrive.start(-60, 0, 0.4);
            //while(!controlledDrive.endReached() && opModeIsActive()) {}
            //controlledDrive.stop();

            backTillButtons();
        }

        // you are back at the wall


        if (opModeIsActive()) {
            omniWheel.setMotors(0.0, 0.4, 0);
            while (!colorTools.isRed(robot.color_back) && opModeIsActive()) {}
            omniWheel.setMotors(0, 0, 0);

        }


        // you are now below the bridge


        if (opModeIsActive()) {
            controlledDrive.start(0, 30, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }

        // yay you should now be at A4, besides the bridge

        if (opModeIsActive()) {
            controlledDrive.start(-generalTools.ap_underBridgeForward, 0, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }

        // you are now standing at B4 congratulation!

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

        // you have now released the stone!

        if (opModeIsActive()) {
            while (!colorTools.isRed(robot.color_back) && opModeIsActive()) {
                omniWheel.setMotors(0, -0.2, 0);
            }
            omniWheel.setMotors(0, 0, 0);
        }

        if (opModeIsActive()) {
            backTillButtons();
        }

        // you are now below the bridge aye


        //if (opModeIsActive()) {
        //    generalTools.stopForMilliSeconds(10000);
        //}

        // you have now waited 10 sec

        if (opModeIsActive()) {
            controlledDrive.start(generalTools.ap_underBridgeForward, 0, 0.2);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you are now standing at B3

    }


    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }
}