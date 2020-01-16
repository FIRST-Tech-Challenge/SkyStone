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

@Autonomous (name = "A2_Red_Bridge")

public class A2_Red_Bridge extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;

    double extenderEncoderValue = 3.5;
    double liftEncoderValue = 1.5;
    double liftStartOffset = 0.75;

    @Override
    public void runOpMode() {

        colorTools = new ColorTools();
        robot = new HardwareChassis(hardwareMap);
        controlledDrive = new ControlledDrive(robot, telemetry);
        generalTools = new GeneralTools(this, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, telemetry);
        controlledExtender = new ControlledExtender(robot, telemetry);

        generalTools.releaseFoundation();
        controlledLift.start(-liftEncoderValue,0.2);

        waitForStart();

        if (opModeIsActive()){
            controlledExtender.start(extenderEncoderValue,0.4);
            controlledLift.start(liftEncoderValue + liftStartOffset,0.2);
        }

        if (opModeIsActive()){
            while (!colorTools.isRed(robot.color_back) && opModeIsActive()){
                /*if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, -0.3, 0);
                } */

                omniWheel.setMotors(0.0, 0.3, 0);
                //backTillButtons();
                //omniWheel.setMotors(0,-1,0);
            }
            omniWheel.setMotors(0,0,0);
        }

        backTillButtons();

    }

    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }


}
