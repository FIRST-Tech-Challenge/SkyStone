package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.ColorTools;
import org.firstinspires.ftc.teamcode.Library.GeneralTools;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

@Autonomous (name = "C_Autonomous_RedTest")

public class ConceptAutonomousRedTest extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;

    @Override
    public void runOpMode() {
        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);

        robot.servo_grab.setPosition(0.6);

        waitForStart();

        if (opModeIsActive() && !colorTools.isRed(robot.color_back)) {
            controlledDrive.start(70, 0, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(500);
            robot.servo_grab.setPosition(1);
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
    }

    private void backTillButtons() {
        omniWheel.setMotors(-0.2, 0, 0);
        while(robot.touch_right.getState() && robot.touch_left.getState()) {}
        omniWheel.setMotors(0, 0, 0);
    }
}
