package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad1;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad1NoWrist;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeNoWrist;

/**
 * TeleOpMode for team Hazmat
 */
@Disabled
@TeleOp(name = "HazmatRobot2DriveOpmode", group = "Teleop")
public class HazmatRobot2DriveOpmode extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepad1NoWrist hzGamepad1NoWrist;
    Chassis hzChassis;
    Arm hzArm;
    IntakeNoWrist hzIntakeNoWrist;

    @Override
    public void runOpMode() {

        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        //hzArm = new Arm(hardwareMap);
        hzIntakeNoWrist = new IntakeNoWrist(hardwareMap);
        hzGamepad1NoWrist = new HzGamepad1NoWrist(gamepad1);

        telemetry.addData("Init", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            hzGamepad1NoWrist.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntakeNoWrist);
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

        /*telemetry.addData("armMotor.isBusy : ", hzArm.armMotor.isBusy());
        telemetry.addData("armMotor.getTargetPosition : ", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition : ", hzArm.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getMode : ", hzArm.armMotor.getMode());
        telemetry.addData("Arm.currentLevel : ", hzArm.currentLevel);
        telemetry.addData("Arm.blockLevel[hzArm.currentLevel] : ", hzArm.blockLevel[hzArm.currentLevel]);
        */


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
