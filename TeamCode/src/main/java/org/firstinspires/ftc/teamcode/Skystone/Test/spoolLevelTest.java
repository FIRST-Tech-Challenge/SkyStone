package org.firstinspires.ftc.teamcode.Skystone.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Skystone.Robot;

@TeleOp(name="spoolLevelTest", group="Linear Opmode")
public class spoolLevelTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        resetRobot();
        robot.initServos();

        waitForStart();

        robot.moveOuttakeToStoneLevel(2);

        while(opModeIsActive()) {
            robot.moveOuttake();

            telemetry.addLine("current spool position: " + robot.getOuttakeSpool().getCurrentPosition());
            telemetry.addLine("spool target: " + robot.spoolTargetEncoderTick);
            telemetry.update();
        }
    }

    private void resetRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

        robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getClampPivot().setDirection(Servo.Direction.FORWARD);

        robot.getOuttakeSpool().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getOuttakeSpool2().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.getIntakeLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
