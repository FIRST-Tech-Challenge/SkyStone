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
@Autonomous (name = "A5_Blue_Foundation_Bridge")
public class A5_Blue_Foundation_Bridge extends LinearOpMode {

    HardwareChassis robot;
    ColorTools colorTools;
    ControlledDrive controlledDrive;
    GeneralTools generalTools;
    OmniWheel omniWheel;
    ControlledLift controlledLift;
    ControlledExtender controlledExtender;

    double extenderEncoderValue = 3.5;
    double liftEncoderValue = 0.5;
    double liftStartOffset = 0.75;
    double liftFoundationValue = 1;
    double extenderFoundationValue = 4;

    @Override
    public void runOpMode(){

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
            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()){
                omniWheel.setMotors(0,-0.2,0);
            }
            omniWheel.setMotors(0,0,0);
        }

        //now you are in the corner

        if (opModeIsActive()){
            backTillButtons();
        }

        // you are now back at the wall

        if (opModeIsActive()){
            controlledExtender.start(extenderFoundationValue, 0.3);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }

        //now the arm of the robot has the right position, to see the foundation

        if (opModeIsActive()) {
            controlledLift.start(-liftFoundationValue,0.2);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // the lift is now back to the start 0 level, to see the foundation

        if (opModeIsActive() && !colorTools.isBlue(robot.color_front)){
            while (!colorTools.isBlue(robot.color_front) && opModeIsActive()){
                omniWheel.setMotors(0.3,0,0);
            }
            omniWheel.setMotors(0,0,0);
        }

        //now you are in front of the foundation

        if (opModeIsActive()){
            generalTools.grabFoundation();
        }

        //now you grabbed the foundation

        generalTools.stopForMilliSeconds(1000);

        if (opModeIsActive()){
            backTillButtons();
        }
        //now you have pulled the foundation in the corner

        if (opModeIsActive()){
            generalTools.releaseFoundation();
        }
        //now you have released the foundation

        if (opModeIsActive()){
            controlledDrive.start(0,80,0.3);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

        }

        // now you're next to the foundation
        // it's save to lower the lift

        if (opModeIsActive()) {
            controlledLift.start(-liftFoundationValue,1);
            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();
        }

        if (opModeIsActive()){
            while (!colorTools.isBlue(robot.color_back) && opModeIsActive()){
                omniWheel.setMotors(0,0.3,0);
            }
            omniWheel.setMotors(0,0,0);
        }
        //now you are below the bridge


    }
    private void backTillButtons() {
        while((robot.touch_right.getState() && robot.touch_left.getState()) || (robot.touch_right.getState() && !robot.touch_left.getState()) || (!robot.touch_right.getState() && robot.touch_left.getState())) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }

}
