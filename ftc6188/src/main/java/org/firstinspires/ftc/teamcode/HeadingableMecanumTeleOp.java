package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.robot.HeadingableMecanumRobot;

@TeleOp (name="Headingable Mecanum TeleOp", group="Headingable")
public class HeadingableMecanumTeleOp extends OpMode
{
    private static final double HEADING_COEFF = 0.01;
    private HeadingableMecanumRobot robot;

    private double desiredHeading = 0;
    private ElapsedTime rotationTimer;
    private ElapsedTime elapsedTime;

    @Override
    public void init()
    {
        robot = new HeadingableMecanumRobot(this.hardwareMap);
        rotationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start()
    {
        elapsedTime.reset();
    }

    @Override
    public void loop()
    {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

        robot.drivetrain.halfPowerInput(gamepad1.right_stick_button);

        desiredHeading += -gamepad1.left_stick_x*rotationTimer.time()*HEADING_COEFF;
        rotationTimer.reset();

        if (gamepad1.left_bumper)
        {
            desiredHeading += -Math.PI/4;
        }
        if (gamepad1.right_bumper)
        {
            desiredHeading += Math.PI/4;
        }
        if (gamepad1.y)
        {
            robot.drivetrain.setExtrinsicOffset(desiredHeading);
        }

        //toggles the robots headingless ability maybe
        if (gamepad1.left_stick_button)
        {
            robot.drivetrain.setExtrinsic(!robot.drivetrain.getExtrinsic());
        }

        robot.drivetrain.setCourse(course);
        robot.drivetrain.setVelocity(velocity);
        robot.drivetrain.setTargetHeading(desiredHeading);
        robot.drivetrain.updateHeading();

        telemetry.addData("Status", "Loop: " + rotationTimer.toString());
        telemetry.addData("Headingless", robot.drivetrain.getExtrinsic());
        telemetry.addData("Course", course);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Rotation", -gamepad1.left_stick_x);
        telemetry.addData("Current Heading", robot.controller.getSensorValue());
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Extrinsic Offset", robot.drivetrain.getExtrinsicOffset());
        telemetry.update();
    }
}
