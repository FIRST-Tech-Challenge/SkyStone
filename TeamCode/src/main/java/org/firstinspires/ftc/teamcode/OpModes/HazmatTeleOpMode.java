package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.*;

/**
 * TeleOpMode for team Hazmat
 */
@TeleOp(name = "HazmatTeleOpMode", group = "Teleop")
public class HazmatTeleOpMode extends LinearOpMode{

    //Trying gamepad without extension

    HzGamepad1 hzGamepad1;

    @Override
    public void runOpMode() {

        //Instantiate controller with gamepad1 connected to it.
        //Controller hzController = new Controller(gamepad1);



        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        Chassis hzChassis = new Chassis(hardwareMap);
        Arm hzArm = new Arm(hardwareMap);
        Intake hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        //Initialize Subsystems - Chassis, Arm, Intake.
        hzChassis.initChassis();
        hzArm.initArm();
        hzIntake.initIntake();

        telemetry.addData("Init", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Tonctoller inputs
        while (opModeIsActive()) {
            hzGamepad1.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntake);
            telemetry.update();
            }
    }
}
