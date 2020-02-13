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

@Disabled
@Autonomous (name = "C_Autonomous_BlueTest")

public class ConceptAutonomousBlueTest extends LinearOpMode {

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
    double liftFoundationValue = 1;

    @Override
    public void runOpMode() {
        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, telemetry);
        controlledExtender = new ControlledExtender(robot, telemetry);

        waitForStart();

        if (opModeIsActive()){
            generalTools.openClamp();
            generalTools.releaseFoundation();
            controlledLift.start(liftEncoderValue, 0.2);
        }

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
            omniWheel.setMotors(0.0, -0.4, 0);
            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()) {}
            omniWheel.setMotors(0, 0, 0);

        }


        // you are now below the bridge


        if (opModeIsActive()) {
            backTillButtons();

            /*
            controlledDrive.start(0, 10, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            */

            omniWheel.setMotors(0.0, -0.4, 0);

            generalTools.stopForMilliSeconds(1000);

            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()) {
                if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, -0.3, 0);
                }
            }
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now in the corner
        // you are now as much as possible close to the wall

        if (opModeIsActive()) {
            controlledExtender.start(extenderFoundationValue, 0.3);

            controlledLift.start(liftFoundationValue, 0.4);

            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }

        // you have now lifted the lift up and extended the arm

        if (opModeIsActive()) {
            while (!colorTools.isBlue(robot.color_front) && opModeIsActive()) {
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
            generalTools.stopForMilliSeconds(500);
        }

        //you have now grabbed the foundation

        if (opModeIsActive()) {
            backTillButtons();
        }

        // you have now dragged the foundation into the corner (in best case)

        if (opModeIsActive()){
            generalTools.releaseFoundation();
        }

        // you have now released the foundation

        if (opModeIsActive()) {
            controlledDrive.start(0, 80, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            /*
            while (colorTools.isRed(robot.color_front)){
                omniWheel.setMotors(0, -0.2, 0);
            }
            omniWheel.setMotors(0,0,0); */
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
            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()){
                if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, 0.2, 0);
                }
            }

            omniWheel.setMotors(0, 0, 0);
        }

        //you are now parked under the bridge
    }


    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState() && opModeIsActive()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }
}