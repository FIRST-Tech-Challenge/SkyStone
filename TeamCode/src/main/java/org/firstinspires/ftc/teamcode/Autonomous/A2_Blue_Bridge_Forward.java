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

@Autonomous (name =  "A2_Blue_Bridge_Forward")

public class A2_Blue_Bridge_Forward extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;

    double extenderEncoderValue = 3.5;
    double liftEncoderValue = 1.5;
    double liftFoundationValue = 1;
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

        waitForStart();

        if (opModeIsActive()) {
            controlledLift.start(liftFoundationValue,0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have noe uplifted the lift

        if (opModeIsActive()){
            controlledExtender.start(extenderEncoderValue,0.4);
            while (!controlledExtender.endReached()) {}
            controlledExtender.stop();
            controlledLift.start(-(liftEncoderValue + liftStartOffset),0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have now lowered the lift and pulled out the arm

        if (opModeIsActive()){
            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()){
                omniWheel.setMotors(0.0, -0.3, 0);
            }
            omniWheel.setMotors(0,0,0);
        }

        // you are now below the bridge

        if (opModeIsActive()){
            backTillButtons();
        }

        // you are now touching the wall

        if (opModeIsActive()) {
            controlledDrive.start(50, 0, 0.2);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
        }

        // you are now below the bridge at B3

    }

    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }



}
