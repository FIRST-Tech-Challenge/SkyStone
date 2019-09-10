package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Test;
/**
 * Created by Khue on 11/10/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;


@TeleOp(name="EncoderRetrieval", group="Linear Opmode")
@Disabled
public class EncoderRetrieval extends LinearOpMode {

    boolean changedBlocker = false, onBlock = false;
    boolean changedHook = false, onHook = false;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    public static double powerScaleFactor = 1.5;
    long startTime = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Init's robot
        RR2 robot = new RR2(hardwareMap, telemetry, this);   //DO NOT DELETE
        robot.resetEncoders();
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            telemetry.addLine("Left Value: " + robot.fLeft.getCurrentPosition());
            telemetry.addLine("Right Value: " + robot.fRight.getCurrentPosition());
            telemetry.update();
        }
    }
}