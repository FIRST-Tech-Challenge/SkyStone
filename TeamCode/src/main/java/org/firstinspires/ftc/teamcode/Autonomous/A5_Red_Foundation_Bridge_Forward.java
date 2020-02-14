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

@Autonomous (name = "A5_Red_Foundation_Bridge_Forward")
public class A5_Red_Foundation_Bridge_Forward extends LinearOpMode {

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
    double liftEncoderValue = 0.5;
    double liftStartOffset = 0.75;
    double liftFoundationValue = 1.6;
    double extenderFoundationValue = 4;
    double startPos;

    @Override
    public void runOpMode(){

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

        if (opModeIsActive()){
            generalTools.releaseFoundation();
        }

        if (opModeIsActive()) {
            controlledLift.start(liftFoundationValue,0.4);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // you have noe uplifted the lift

        if (opModeIsActive()){
            orientationTools.driveSidewardEncoder(this, 0, 50, 0.6, omniWheel, startPos, robotGyro.imu, 175, 150);
        }

        //now you are in the corner

        if (opModeIsActive()){
            controlledExtender.start(extenderFoundationValue, 0.4);
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }

        //now the arm of the robot has the right position, to see the foundation

        if (opModeIsActive()) {
            controlledLift.start(-liftFoundationValue,0.4);
            while (!controlledLift.endReached()) {}
            controlledLift.stop();
        }

        // the lift is lifted

        if (opModeIsActive()) {
            controlledDrive.start(80, 0, 0.2);
            while (!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
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

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -90, -0.4, omniWheel, startPos, robotGyro.imu, 175, 150); //sideways: 220 --> middle
        }

        // now you're next to the foundation
        // it's save to lower the lift

        if (opModeIsActive()) {
            controlledLift.start(-liftStartOffset,1);
            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();
        }

        if (opModeIsActive()) {
            controlledDrive.start(generalTools.ap_underBridgeForward, 0, 0.4);
            while (!controlledDrive.endReached() && opModeIsActive()) { }
            controlledDrive.stop();
        }

        if (opModeIsActive()) {
            orientationTools.driveSidewardEncoder(this, 0, -40, -0.4, omniWheel, startPos, robotGyro.imu, 175, 150); //sideways: 220 --> middle
        }


    }
    private void backTillButtons() {
        while((robot.touch_right.getState() && robot.touch_left.getState()) || (robot.touch_right.getState() && !robot.touch_left.getState()) || (!robot.touch_right.getState() && robot.touch_left.getState())) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }

}
