package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@Autonomous (name = "C_Autonomous_RedTest")

public class ConceptAutonomousRedTest extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;

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

        waitForStart();

        if (opModeIsActive()) {
            controlledLift.start(15, 0.2);
        }

        if (opModeIsActive()) {
            controlledExtender.start(15, 0.2);
        }
        
        if (opModeIsActive() && !colorTools.isRed(robot.color_back)) {
            controlledDrive.start(70, 0, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(500);
            generalTools.closeClamp();
        }

        // hey... you should have grabbed a stone now...

        if (opModeIsActive()) {
            controlledDrive.start(-60, 0, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            backTillButtons();
        }

        // you are back at the wall

        if (opModeIsActive()) {
            omniWheel.setMotors(0.0, 0.1, 0);
            while (!colorTools.isRed(robot.color_back) && opModeIsActive()) {}
            omniWheel.setMotors(0, 0, 0);
        }

        // you are now below the bridge

        if (opModeIsActive()) {
            backTillButtons();

            controlledDrive.start(0, 10, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            omniWheel.setMotors(0.0, 0.1, 0);
            while (!colorTools.isRed(robot.color_back) && opModeIsActive()) {
                if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, 0.1, 0);
                }
            }
            omniWheel.setMotors(0, 0, 0);
        }

            // you are now in the corner
            // you are now as much as possible close to the wall
        if (opModeIsActive()) {
            controlledLift.start(15, 0.2);
        }

            // you have now lifted the lift up

        if (opModeIsActive()) {
            controlledExtender.start(15, 0.2);
        }

            // you have now your arm out


        if (opModeIsActive()) {
            while (!colorTools.isRed(robot.color_front)) {
                omniWheel.setMotors(0.2, 0, 0);
            }
        }

            // you are now standing right in front of the foundation

        if (opModeIsActive()) {
            generalTools.openClamp();
        }

            // you have now released the stone on the foundation

        if (opModeIsActive()) {
            generalTools.grabFoundation();
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
            controlledExtender.start(-15, 0.2);
        }

            // you have now put the arm back in

        if (opModeIsActive()) {
            controlledLift.start(-20, 0.2);
        }

            // you have now lifted the lift down

        if (opModeIsActive()) {
            while (!colorTools.isRed(robot.color_back)){
                omniWheel.setMotors(0, -0.2, 0);
            }
            omniWheel.setMotors(0, 0, 0);
        }

            //you are now parked under the bridge
    }






    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState()) {
            omniWheel.setMotors(-0.2, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }
}



