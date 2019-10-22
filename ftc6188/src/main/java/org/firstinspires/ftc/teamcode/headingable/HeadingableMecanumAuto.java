package org.firstinspires.ftc.teamcode.headingable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.robot.HeadingableMecanumRobot;

@Disabled
@Autonomous (name="Headingable Mecanum Auto", group="TeleHead")
public class HeadingableMecanumAuto extends LinearOpMode
{
    private HeadingableMecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new HeadingableMecanumRobot(this.hardwareMap, true);

        while (!robot.imu.isGyroCalibrated());
        waitForStart();

        robot.drivetrain.setTargetHeading(Math.PI/2);
        while(robot.drivetrain.isRotating())
        {
            robot.drivetrain.updateHeading();
            doTelemetry();
        }
        sleep(1000);

        robot.drivetrain.setTargetHeading(-Math.PI/2);
        while(robot.drivetrain.isRotating())
        {
            robot.drivetrain.updateHeading();
            telemetry.addData("Heading", robot.drivetrain.getCurrentHeading());
            telemetry.update();
        }
        sleep(1000);

        while (opModeIsActive())
        {
            robot.drivetrain.updateHeading();
        }
    }

    private void doTelemetry()
    {
        PIDController pid = (PIDController) robot.drivetrain.controller.algorithm;
        telemetry.addData("Heading, Target", robot.drivetrain.controller.getSensorValue() + "," + pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.update();
    }
}
