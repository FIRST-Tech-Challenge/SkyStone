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
@Autonomous (name =  "A2_Blue_Bridge")

public class A2_Blue_Bridge extends LinearOpMode {

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
    double extenderFoundationValue = 4;



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
            controlledLift.start(generalTools.liftFoundationValue,0.2);
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

        //you are now touching the wall behind
    }
    private void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }

}
