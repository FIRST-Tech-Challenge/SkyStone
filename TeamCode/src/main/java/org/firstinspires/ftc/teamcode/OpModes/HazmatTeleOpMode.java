package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.*;

/**
 * TeleOpMode for team Hazmat
 */
@TeleOp(name = "HazmatTeleOpMode", group = "Teleop")
public class HazmatTeleOpMode extends LinearOpMode{

    public boolean HzDEBUG_FLAG = true;

    int hzRobotNum; //Set to 1 for main robot, 2 for second robot

    HzGamepad1 hzGamepad1;
    Chassis hzChassis;
    Arm hzArm;
    Intake hzIntake;


    @Override
    public void runOpMode() {

        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        telemetry.addData("Init", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            hzGamepad1.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntake);
            if(HzDEBUG_FLAG) printDebugMessages();
            telemetry.update();
            }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("armMotor.isBusy : ", hzArm.armMotor.isBusy());
        telemetry.addData("armMotor.getTargetPosition : ", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition : ", hzArm.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getMode : ", hzArm.armMotor.getMode());
        telemetry.addData("Arm.currentLevel : ", hzArm.currentLevel);
        telemetry.addData("Arm.blockLevel[hzArm.currentLevel] : ", hzArm.blockLevel[hzArm.currentLevel]);



        /*
        telemetry.addData("Intake.grip.getPosition", hzIntake.grip.getPosition());
        telemetry.addData("Intake.wrist.getPosition", hzIntake.wrist.getPosition());
        telemetry.addData("hzGamepad1.gpGamepad1.dpad_up", hzGamepad1.gpGamepad1.dpad_up);
        telemetry.addData("hzGamepad1.getDpad_upPress", hzGamepad1.getDpad_upPress());
        telemetry.addData("hzGamepad1.gpGamepad1.dpad_down", hzGamepad1.gpGamepad1.dpad_down);
        telemetry.addData("hzGamepad1.getDpad_downPress", hzGamepad1.getDpad_downPress());
        telemetry.addData("Intake.grip.getPosition", hzIntake.wrist.getPosition());
        telemetry.addData("Chassis.touchSensorIsPressed", hzChassis.frontleftChassisTouchSensorIsPressed());
        */
    }

}


