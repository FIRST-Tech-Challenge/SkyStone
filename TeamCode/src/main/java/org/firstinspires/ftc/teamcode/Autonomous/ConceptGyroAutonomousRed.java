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
@Autonomous(name = "C_Autonomous_Gyro")

public class ConceptGyroAutonomousRed extends LinearOpMode {

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

        generalTools.openClamp();
        generalTools.releaseFoundation();
        controlledLift.start(liftEncoderValue, 0.2);

        waitForStart();
        controlledLift.stop();

        if (opModeIsActive() ) {
            controlledExtender.start(extenderEncoderValue, 0.4);

            controlledDrive.start(generalTools.ap_forwardGrabStone, 0, 0.4);
            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            while(!controlledExtender.endReached() && opModeIsActive()) {}
            controlledExtender.stop();

            controlledLift.start(-(liftEncoderValue + liftStartOffset), 0.2); //lowers the lift
            generalTools.stopForMilliSeconds(500);

            while(!controlledDrive.endReached() && opModeIsActive()) {}
            controlledDrive.stop();

            generalTools.stopForMilliSeconds(1000);
            generalTools.closeClamp();

            generalTools.stopForMilliSeconds(500);
        }

        // hey... you should have grabbed a stone now...


    }
}