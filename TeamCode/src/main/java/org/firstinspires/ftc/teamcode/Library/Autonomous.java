package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledExtender;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;

public class Autonomous {
    LinearOpMode opMode;

    public ColorEnum color;

    public HardwareChassis robot;
    public ColorTools colorTools;
    public ControlledDrive controlledDrive;
    public GeneralTools generalTools;
    public OmniWheel omniWheel;
    public ControlledLift controlledLift;
    public ControlledExtender controlledExtender;

    double extenderEncoderValue = 3.5;
    double extenderFoundationValue = 4;
    double liftEncoderValue = 1.5;
    double liftStartOffset = 0.5;
    double liftFoundationValue = 1;
    double driveToRandomStoneValue = 65;
    double side;

    public Autonomous(LinearOpMode opMode, ColorEnum color) {
        this.opMode = opMode;

        this.color = color;

        colorTools = new ColorTools();
        robot = new HardwareChassis(opMode.hardwareMap);
        controlledDrive = new ControlledDrive(robot, opMode.telemetry);
        generalTools = new GeneralTools(opMode, robot);
        omniWheel = new OmniWheel(robot);
        controlledLift = new ControlledLift(robot, opMode.telemetry);
        controlledExtender = new ControlledExtender(robot, opMode.telemetry);

        side = 1;
        if (color == ColorEnum.Blue) {
            side = -1;
        }
    }

    public boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    public void initServos() {
        generalTools.openClamp();
        generalTools.releaseFoundation();
    }

    public void initLiftPosition() {
        controlledLift.start(liftEncoderValue, 0.2);
    }

    public void stopInitLiftPosition() {
        if (opModeIsActive() ) {
            controlledLift.stop();
        }
    }

    public void prepareExtender() {
        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.5);
        }
    }

    public void driveToRandomStone() {
        if (opMode.opModeIsActive()) {
            controlledDrive.start(65, 0, 0.4);

            while (!controlledExtender.endReached() && opModeIsActive()) {
            }
            controlledExtender.stop();
        }
    }

    public void grabQuarryStone() {
        if (opMode.opModeIsActive()) {
            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.2); //lowers the lift

            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(1000);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }
    }

    public void driveBackToWall() {
        if (opModeIsActive()) {
            backTillButtons();
        }
    }

    public void driveToBridge() {
        if (opModeIsActive()) {
            omniWheel.setMotors(0.0, 0.4 * side, 0);
            while (!isTeamColor(robot.color_back) && opModeIsActive()) {}
            omniWheel.setMotors(0, 0, 0);
        }
    }

    public void driveToBuildingSite() {
        if (opModeIsActive()) {
            backTillButtons();

            omniWheel.setMotors(0.0, 0.4 * side, 0);

            generalTools.stopForMilliSeconds(1000);

            while (!isTeamColor(robot.color_back) && opModeIsActive()) {
                if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, 0.3 * side, 0);
                }
            }
            omniWheel.setMotors(0, 0, 0);
        }
    }

    public void prepareArmForFoundation() {
        if (opModeIsActive()) {
            controlledExtender.start(extenderFoundationValue, 0.3);

            controlledLift.start(liftFoundationValue, 0.4);

            while(!controlledLift.endReached() && opModeIsActive()) {}
            controlledLift.stop();
            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();
        }
    }

    public void driveToFoundation() {
        if (opModeIsActive()) {
            while (!isTeamColor(robot.color_front) && opModeIsActive()) {
                omniWheel.setMotors(0.4, 0, 0);
            }
            omniWheel.setMotors(0, 0, 0);
        }
    }

    public void driveTowardsBridgeFromBuildingSite() {
        if (opModeIsActive()) {
            controlledDrive.start(0, (-80) * side, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();
            /*
            while (colorTools.isRed(robot.color_front)){
                omniWheel.setMotors(0, -0.2, 0);
            }
            omniWheel.setMotors(0,0,0); */
        }
    }

    public void foldIn() {
        if (opModeIsActive()) {
            controlledExtender.start(-extenderFoundationValue, 0.2);
            controlledLift.start(-liftFoundationValue, 0.2);

            while(!controlledExtender.endReached() && opModeIsActive()) {}
            while(!controlledLift.endReached() && opModeIsActive()) {}
            
            controlledExtender.stop();
            controlledLift.stop();

            // you have now put the arm and extender back in
        }
    }

    public void parkUnderBridge() {
        if (opModeIsActive()) {
            while (!isTeamColor(robot.color_back) && opModeIsActive()){
                if (robot.touch_right.getState() && robot.touch_left.getState()) {
                    omniWheel.setMotors(-0.1, 0, 0);
                    while (robot.touch_right.getState() && robot.touch_left.getState()) {}
                    omniWheel.setMotors(0.0, (-0.2) * side, 0);
                }
            }
            omniWheel.setMotors(0, 0, 0);
        }
    }


    public void backTillButtons() {
        while(robot.touch_right.getState() && robot.touch_left.getState() && opModeIsActive()) {
            omniWheel.setMotors(-0.3, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }

    public boolean isTeamColor(ColorSensor sensor) {
        if (color == ColorEnum.Blue) {
            return colorTools.isBlue(sensor);
        } else {
            return colorTools.isRed(sensor);
        }
    }
}
