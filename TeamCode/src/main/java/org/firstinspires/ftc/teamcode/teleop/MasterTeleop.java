package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.hardwareutils.HardwareManager;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TwinstickMecanum;
import org.firstinspires.ftc.teamcode.subsystems.subsystemutils.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.subsystemutils.SubsystemManager;

@TeleOp
public class MasterTeleop extends OpMode {
    HardwareManager hardware;

    Gamepad driveController; //gamepad 1;
    Gamepad manipController; //gamepad 2;

    SubsystemManager subsystems;

    @Override
    public void init() {
        //verify switch on bottom is in X pos
        //for drive controller, do Start btn + A btn
        //for manip controller, do Start btn + B btn
        hardware = new HardwareManager(hardwareMap);
        driveController = gamepad1;
        manipController = gamepad2;

        Subsystem drive = setUpDriveTrain();
        Subsystem elevator = setUpElevator();
        Subsystem grabber = setupGrabber();
        Subsystem intake = setupIntake();

        subsystems = new SubsystemManager(drive, elevator, grabber, intake);
        subsystems.init();

    }

    @Override
    public void loop() {
        telemetry.addData("status", "loop");
        telemetry.update();
        subsystems.update();
    }

    private Subsystem setUpDriveTrain() {
        return new TwinstickMecanum(
                driveController,
                hardware.leftFrontDrive,
                hardware.rightFrontDrive,
                hardware.leftRearDrive,
                hardware.rightRearDrive
        );
    }

    private Subsystem setUpElevator() {
        return new Elevator(
                manipController,
                hardware.elevatorMotor
        );
    }

    private Subsystem setupIntake() {
        return new Intake(
                driveController,
                hardware.boot,
                hardware.leftIntakeMotor,
                hardware.rightIntakeMotor
        );
    }

    private Subsystem setupGrabber() {
        return new Grabber(
                manipController,
                hardware.latch,
                hardware.blockPanServo
        );
    }
}
